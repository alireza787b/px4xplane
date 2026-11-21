"""Check staged documentation without building PX4 or requiring a Markdown package."""
from html.parser import HTMLParser
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]


class HtmlLinks(HTMLParser):
    def __init__(self):
        super().__init__()
        self.urls = []

    def handle_starttag(self, tag, attrs):
        self.urls.extend(value for key, value in attrs if key in {'href', 'src'} and value)


def missing_links(root):
    """Check inline Markdown/image links and raw HTML links used by our docs."""
    missing = []
    for page in sorted(root.rglob('*.md')):
        text = page.read_text(encoding='utf-8')
        html = HtmlLinks()
        html.feed(text)
        urls = re.findall(r'\]\(([^\s)]+)(?:\s+"[^"]*")?\)', text) + html.urls
        for url in urls:
            parsed = urlsplit(url)
            if parsed.scheme or parsed.netloc or not parsed.path:
                continue
            target = (page.parent / unquote(parsed.path)).resolve()
            if not target.is_relative_to(root.resolve()) or not target.exists():
                missing.append(f'{page.relative_to(root)} -> {url}')
    return missing


class PackageDocs(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.package = Path(self.temp.name) / 'package with spaces'
        shutil.copytree(ROOT / 'docs', self.package / 'docs')
        shutil.copytree(ROOT / 'config/px4_params', self.package / 'px4_airframes')
        shutil.copy2(ROOT / 'README.md', self.package / 'README.md')
        (self.package / '64').mkdir()
        shutil.copy2(ROOT / 'config/config.ini', self.package / '64/config.ini')

    def stage(self):
        subprocess.run(['cmake', f'-DSOURCE_ROOT={ROOT}', f'-DPLUGIN_ROOT={self.package}',
                        '-P', str(ROOT / 'cmake/PackageDocs.cmake')],
                       check=True, capture_output=True, text=True)

    def test_all_packaged_markdown_links_resolve_inside_package(self):
        self.stage()
        self.assertEqual(missing_links(self.package), [])
        profiles = list(self.package.rglob('[0-9][0-9][0-9][0-9]_xplane_*'))
        self.assertEqual(len(profiles), 5)
        self.assertTrue(all(p.parent == self.package / 'px4_airframes' for p in profiles))
        self.assertFalse(any(p.is_symlink() for p in self.package.rglob('*')))
        self.assertEqual((self.package / '64/config.ini').read_bytes(),
                         (ROOT / 'config/config.ini').read_bytes())

    def test_refresh_removes_duplicate_root_cards_and_is_repeatable(self):
        for name in ['ALIA', 'EHANG184', 'QUADTAILSITTER', 'CESSNA172', 'TB2']:
            (self.package / f'{name}_XPLANE12_TEST.md').write_text('stale\n')
        (self.package / 'compact-qtailsitter.md').write_text('stale\n')
        self.stage()
        first = {p.relative_to(self.package): p.read_bytes() for p in self.package.rglob('*.md')}
        self.stage()
        self.assertEqual(first, {p.relative_to(self.package): p.read_bytes()
                                 for p in self.package.rglob('*.md')})
        self.assertEqual(sorted(p.name for p in self.package.glob('*.md')),
                         ['CHANGELOG.md', 'README.md'])

    def test_remapping_preserves_source_links_and_fragments(self):
        source = ROOT / 'docs/QUADTAILSITTER_XPLANE12_TEST.md'
        original = source.read_bytes()
        self.stage()
        card = (self.package / 'docs' / source.name).read_text()
        self.assertIn('](../px4_airframes/5021_xplane_qtailsitter)', card)
        reference = (self.package / 'px4_airframes/README.md').read_text()
        self.assertIn('](../docs/guided-launcher.md#saved-parameters)', reference)
        self.assertEqual(source.read_bytes(), original)

    def test_checker_reports_missing_and_escaping_targets(self):
        self.stage()
        (self.package / 'broken.md').write_text('[missing](missing.md)\n[escape](../outside.md)\n')
        self.assertEqual(missing_links(self.package),
                         ['broken.md -> missing.md', 'broken.md -> ../outside.md'])
