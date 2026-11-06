import math
import unittest
import cv2
import numpy as np
from cv.gauge_reader import Calibration,angle_to_value,read_gauge


def synthetic(angle):
    image=np.full((260,260,3),80,np.uint8)
    cv2.circle(image,(130,130),95,(25,25,25),-1)
    cv2.circle(image,(130,130),88,(240,240,240),-1)
    a=math.radians(angle)
    tip=(round(130+70*math.cos(a)),round(130-70*math.sin(a)))
    tail=(round(130-16*math.cos(a)),round(130+16*math.sin(a)))
    cv2.line(image,tail,tip,(25,25,25),4)
    cv2.circle(image,(130,130),6,(25,25,25),-1)
    return image


class ReaderTest(unittest.TestCase):
    def test_scale_edges_negative_range_and_gap(self):
        c=Calibration(-1,0,'bar')
        self.assertEqual(angle_to_value(225,c),-1)
        self.assertEqual(angle_to_value(-45,c),0)
        self.assertAlmostEqual(angle_to_value(90,c),-.5)
        self.assertIsNone(angle_to_value(270,c))

    def test_generated_2d_angles_tail_and_negative_scale(self):
        c=Calibration(-1,1,'bar')
        for angle in (225,180,90,45,315):
            with self.subTest(angle=angle):
                r=read_gauge(synthetic(angle),c,'synthetic')
                self.assertEqual(r['read_status'],'ok',r)
                self.assertAlmostEqual(r['value'],angle_to_value(angle,c),delta=.04)

    def test_invalid_blank_occluded_explicit_refusal(self):
        c=Calibration(0,10,'bar')
        images=[None,np.zeros((100,100,3),np.uint8),np.ones((100,100,3),np.float32)]
        covered=synthetic(40);cv2.rectangle(covered,(45,45),(215,215),(0,0,0),-1);images.append(covered)
        for image in images:
            result=read_gauge(image,c)
            self.assertIsNone(result['value'])
            self.assertNotEqual(result['read_status'],'ok')
            self.assertTrue(result['error'])

    def test_roi_is_optional_and_strict(self):
        rgb=synthetic(90)
        self.assertEqual(read_gauge(rgb,Calibration(0,10,'bar'),roi=[30,30,200,200])['read_status'],'ok')
        self.assertEqual(read_gauge(rgb,Calibration(0,10,'bar'),roi=[-1,0,100,100])['error'],'invalid_roi')


if __name__=='__main__': unittest.main()
