"""Compile the recorded PX4 math routines, not a replacement allocator model."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

REVISION = '9cfe130a8d8e0c25f88e044141c523247ca53d48'


def method(source, marker):
    start = source.index('{', source.index(marker))
    # PX4 method-level closing braces begin at column zero; nested braces do not.
    return source[start:source.index('\n}', start)+2]


class AllocatorContract(unittest.TestCase):
    def test_recorded_filter_and_normalization(self):
        repo = Path(os.environ.get('PX4_TEST_REPO', str(Path.home()/'PX4-Autopilot-Me')))
        if not shutil.which('g++') or not (repo/'src/lib/matrix').is_dir():
            self.skipTest('Requires g++ and the existing PX4 matrix headers')
        def source(path):
            return subprocess.check_output(['git', '-C', str(repo), 'show', REVISION+':'+path], text=True)
        rotors = source('src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRotors.cpp')
        allocator = source('src/modules/control_allocator/ControlAllocator.cpp')
        inverse = source('src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp')
        filter_start = allocator.index('for (int n = 0; n < NUM_AXES; n++)', allocator.index('// Set all the elements of a row'))
        filtering = allocator[filter_start:allocator.index('// Assign control effectiveness matrix', filter_start)]
        text = '''#include <matrix/math.hpp>
#include <cfloat>
#include <cmath>
#include <cassert>
#include <initializer_list>
using matrix::Vector3f;
using Matrix = matrix::Matrix<float,6,16>;
struct ActuatorEffectivenessRotors {
 static constexpr int NUM_ACTUATORS=16;
 using EffectivenessMatrix=Matrix;
 struct Rotor { Vector3f position, axis; float thrust_coef, moment_ratio; };
 struct Geometry { Rotor rotors[12]; int num_rotors=4;
  bool propeller_torque_disabled=false, propeller_torque_disabled_non_upwards=false;
  bool yaw_by_differential_thrust_disabled=false, three_dimensional_thrust_disabled=false; };
 static int computeEffectivenessMatrix(const Geometry &geometry, EffectivenessMatrix &effectiveness, int actuator_start_index)
''' + method(rotors, 'ActuatorEffectivenessRotors::computeEffectivenessMatrix') + '''
};
class ControlAllocationPseudoInverse {
public:
 static constexpr int THRUST_Z=5, NUM_AXES=6;
 bool _normalize_rpy=true; int _num_actuators=4;
 matrix::Matrix<float,16,6> _mix;
 matrix::Vector<float,6> _control_allocation_scale;
 void updateControlAllocationMatrixScale()
''' + method(inverse, 'ControlAllocationPseudoInverse::updateControlAllocationMatrixScale') + '''
 void normalizeControlAllocationMatrix()
''' + method(inverse, 'ControlAllocationPseudoInverse::normalizeControlAllocationMatrix') + '''
};
Matrix effectiveness(float ct) {
 ActuatorEffectivenessRotors::Geometry g;
 float x[]={.219456f,-.219456f,.219456f,-.219456f};
 float y[]={.42672f,-.42672f,-.42672f,.42672f};
 for(int k=0;k<4;k++) { g.rotors[k].position=Vector3f(x[k],y[k],-.04572f);
  g.rotors[k].axis=Vector3f(0,0,-1); g.rotors[k].thrust_coef=ct;
  g.rotors[k].moment_ratio=(k<2 ? 1.f:-1.f)*.019050555f; }
 Matrix m; m.setZero();
 ActuatorEffectivenessRotors::computeEffectivenessMatrix(g,m,0); return m;
}
void weakFilter(Matrix &matrix) {
 constexpr int NUM_AXES=6;
 struct Config {int num_actuators_matrix[1]={4};} config;
 int i=0;
''' + filtering + '''
}
int main() {
 Matrix old=effectiveness(2), corrected=effectiveness(6.5f);
 assert(fabsf(old(2,0)) < .05f && fabsf(corrected(2,0)) > .05f);
 ControlAllocationPseudoInverse a,b;
 matrix::geninv(old,a._mix); matrix::geninv(corrected,b._mix);
 a.updateControlAllocationMatrixScale(); a.normalizeControlAllocationMatrix();
 b.updateControlAllocationMatrixScale(); b.normalizeControlAllocationMatrix();
 for(int i=0;i<16;i++) for(int j=0;j<6;j++) assert(fabsf(a._mix(i,j)-b._mix(i,j)) < 1e-4f);
 weakFilter(old); weakFilter(corrected);
 for(int k=0;k<4;k++) assert(old(2,k)==0 && fabsf(corrected(2,k))>.05f);
 matrix::Matrix<float,16,6> mix; matrix::geninv(corrected,mix);
 auto projection=corrected*mix;
 for(int k : {0,1,2,5}) assert(fabsf(projection(k,k)-1.f)<1e-4f);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            include = root/'px4_platform_common'
            include.mkdir()
            (include/'defines.h').write_text('#pragma once\n')
            (root/'contract.cpp').write_text(text)
            subprocess.run(['g++', '-std=c++17', '-O2', '-I'+str(root), '-I'+str(repo/'src/lib/matrix'),
                            str(root/'contract.cpp'), '-o', str(root/'contract')], check=True)
            subprocess.run([str(root/'contract')], check=True)


if __name__ == '__main__':
    unittest.main()
