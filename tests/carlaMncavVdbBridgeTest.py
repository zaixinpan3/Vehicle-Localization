"""Frame and sensor contracts independent of a running CARLA server."""
import sys
import unittest
from pathlib import Path
import numpy as np
import carla
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/carla'))
from captureMncavVdbDrive import rotations, synthesize_sensors, Z_FLIP


class VdbBridgeTest(unittest.TestCase):
    def row(self):
        names = ['time','x','y','zDown','roll','pitch','yaw','vx','vyRight','vzDown',
                 'p','q','r','ax','ayRight','azDown','omegaFL','omegaFR','omegaRL','omegaRR','steerFL','steerFR']
        return np.zeros(1, dtype=[(x, float) for x in names])

    def noise(self):
        return dict(accelerometerNoiseStdMps2=0., gyroNoiseStdRadps=0., wheelNoiseStdRadps=0.,
                    steeringNoiseStdRad=0., gnssNoiseStdM=0.)

    def test_full_attitude_matches_carla_matrix(self):
        data=self.row();data['roll']=.13;data['pitch']=-.07;data['yaw']=1.2
        r,rp=rotations(data[0]);a=carla.Transform(rotation=carla.Rotation(roll=np.degrees(.13),pitch=np.degrees(-.07),yaw=np.degrees(1.2)))
        np.testing.assert_allclose(np.array(a.get_matrix())[:3,:3],Z_FLIP@r@Z_FLIP,atol=2e-7)
        np.testing.assert_allclose(rp.T@rp,np.eye(3),atol=1e-14)

    def test_stationary_accelerometer_measures_specific_force(self):
        data=self.row();data['roll']=.12;data['pitch']=-.1
        _,r=rotations(data[0]);s=synthesize_sensors(data,self.noise(),2.3,1)
        np.testing.assert_allclose(r@s[0,1:4],[0,0,9.81],atol=1e-12)

    def test_encoders_do_not_use_truth_speed(self):
        data=self.row();data['vx']=123;data['omegaFL']=4;data['omegaFR']=5;data['omegaRL']=6;data['omegaRR']=7
        s=synthesize_sensors(data,self.noise(),2.3,1)
        np.testing.assert_array_equal(s[0,7:11],[4,5,6,7])
        data['vx']=-87
        np.testing.assert_array_equal(s,synthesize_sensors(data,self.noise(),2.3,1))

    def test_turn_sign_and_output_lever_arm(self):
        data=self.row();data['yaw']=np.pi/2;data['r']=.2;data['ayRight']=2.
        data['steerFL']=.1;data['steerFR']=.1
        s=synthesize_sensors(data,self.noise(),2.3,1)
        self.assertAlmostEqual(s[0,2],-2.);self.assertAlmostEqual(s[0,6],-.2)
        self.assertAlmostEqual(s[0,11],-.1)
        np.testing.assert_allclose(s[0,12:14],[0,2.3],atol=1e-14)

if __name__=='__main__':unittest.main()
