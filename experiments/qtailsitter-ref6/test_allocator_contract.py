"""Compile the recorded PX4 math routines, not a replacement allocator model."""
import os
import re
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
    def test_recorded_fw_frame_and_differential_scale(self):
        repo=Path(os.environ.get('PX4_TEST_REPO', str(Path.home()/'PX4-Autopilot-Me')))
        if not shutil.which('g++') or not (repo/'src/lib/matrix').is_dir():
            self.skipTest('Requires g++ and existing PX4 matrix headers')
        def source(path):
            return subprocess.check_output(['git','-C',str(repo),'show',REVISION+':'+path],text=True)
        fw=source('src/modules/fw_rate_control/FixedwingRateControl.cpp')
        vtol=source('src/modules/vtol_att_control/tailsitter.cpp')
        expressions=re.findall(r'^\s*body_rates_setpoint = Vector3f\([^;]+\);',fw,re.MULTILINE)
        self.assertEqual(len(expressions),1)
        expression=expressions[0]
        start=fw.index('const float helper',fw.index('// Tailsitter: rotate back to body frame'))
        end=fw.index('_vehicle_torque_setpoint.xyz[2] = -helper;',start)+len('_vehicle_torque_setpoint.xyz[2] = -helper;')
        rotate=fw[start:end]
        start=vtol.index('/* allow differential thrust if enabled */',vtol.index('void Tailsitter::fill_actuator_outputs'))
        end=vtol.index('// for the short period after switching to FW',start)
        differential=vtol[start:end]
        cpp='''#include <matrix/math.hpp>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
using matrix::Vector3f;
struct Setpoint {float xyz[3]{};};
template<class T> struct Param {T value; T get() const {return value;}};
enum class VtFwDifthrEnBits {ROLL_BIT=1,PITCH_BIT=2,YAW_BIT=4};
int main() {
 Param<int> _param_vt_fw_difthr_en{7};
 Param<float> _param_vt_fw_difthr_s_r{.55f},_param_vt_fw_difthr_s_p{.90f},_param_vt_fw_difthr_s_y{.32f};
 struct Rates {float roll,pitch,yaw;} _rates_sp{.17f,-.23f,.31f};
 Vector3f body_rates_setpoint;
''' + expression + '''
 const float mc[3][3]={{.031815f,.007954f,.000636f},{.12f,.015f,.0012f},{.041448f,.007875f,.000829f}};
 for(int term=0;term<3;term++) {
  Setpoint _vehicle_torque_setpoint;
  _vehicle_torque_setpoint.xyz[0]=body_rates_setpoint(0)*mc[2][term]/.55f;
  _vehicle_torque_setpoint.xyz[1]=body_rates_setpoint(1)*mc[1][term]/.90f;
  _vehicle_torque_setpoint.xyz[2]=body_rates_setpoint(2)*mc[0][term]/.32f;
''' + rotate + '''
  Setpoint result;
  auto *_torque_setpoint_0=&result;
  auto *_vehicle_torque_setpoint_virtual_fw=&_vehicle_torque_setpoint;
''' + differential + '''
  const float error[]={_rates_sp.roll,_rates_sp.pitch,_rates_sp.yaw};
  for(int axis=0;axis<3;axis++) {
   if(fabsf(result.xyz[axis]-error[axis]*mc[axis][term])>=1e-6f)
    fprintf(stderr,"term %d axis %d got %f expected %f\\n",term,axis,result.xyz[axis],error[axis]*mc[axis][term]);
   assert(fabsf(result.xyz[axis]-error[axis]*mc[axis][term])<1e-6f);
  }
 }
}
'''
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);include=root/'px4_platform_common';include.mkdir()
            (include/'defines.h').write_text('#pragma once\n')
            (root/'frame.cpp').write_text(cpp)
            subprocess.run(['g++','-std=c++17','-O2','-I'+str(root),'-I'+str(repo/'src/lib/matrix'),
                            str(root/'frame.cpp'),'-o',str(root/'frame')],check=True)
            subprocess.run([str(root/'frame')],check=True)

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
Matrix effectiveness(float ct, bool compact=false, bool cant=false) {
 ActuatorEffectivenessRotors::Geometry g;
 float x[]={.219456f,-.219456f,.219456f,-.219456f};
 float y[]={.42672f,-.42672f,-.42672f,.42672f};
 if(compact) for(int k=0;k<4;k++) {x[k]=(x[k]>0 ? .15f:-.15f);y[k]=(y[k]>0 ? .3f:-.3f);}
 for(int k=0;k<4;k++) { g.rotors[k].position=Vector3f(x[k],y[k],-.04572f);
  float ax[]={-.078f,.078f,-.078f,.078f}, ay[]={.040f,-.040f,-.040f,.040f};
  g.rotors[k].axis=cant ? Vector3f(ax[k],ay[k],-.996f):Vector3f(0,0,-1); g.rotors[k].thrust_coef=ct;
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
 Matrix compact=effectiveness(6.5f,true); weakFilter(compact);
 ControlAllocationPseudoInverse c;
 matrix::geninv(compact,c._mix);
 c.updateControlAllocationMatrixScale(); c.normalizeControlAllocationMatrix();
 // PX4 uses a shared roll/pitch normalization: different arm lengths do NOT
 // imply twice the pitch moment per normalized command for this rectangle.
 auto normalized=compact*c._mix;
 const float expected_rp=2.f*sqrtf(2.f)*.15f*6.5f;
 assert(fabsf(normalized(0,0)-expected_rp)<1e-4f);
 assert(fabsf(normalized(1,1)-expected_rp)<1e-4f);
 assert(fabsf(normalized(2,2)-4.f*.019050555f*6.5f)<1e-4f);
 Matrix restored=effectiveness(6.5f,true,true); weakFilter(restored);
 matrix::Matrix<float,16,6> restored_mix; matrix::geninv(restored,restored_mix);
 auto restored_projection=restored*restored_mix;
 // Four motors cannot independently control all six wrench components.
 // Prove the torque/thrust-Z subset itself remains full rank after restoration.
 for(int k : {0,1,2,5}) {
   assert(restored.row(k).norm_squared()>0);
 }
 matrix::Matrix<float,4,4> subset;
 int axes[]={0,1,2,5};
 for(int i=0;i<4;i++) for(int j=0;j<4;j++) subset(i,j)=restored(axes[i],j);
 matrix::Matrix<float,4,4> subset_inverse; matrix::geninv(subset,subset_inverse);
 auto identity=subset*subset_inverse;
 for(int i=0;i<4;i++) assert(fabsf(identity(i,i)-1.f)<1e-4f);
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
