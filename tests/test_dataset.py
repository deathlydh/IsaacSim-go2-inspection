import unittest
import json
import tempfile
from pathlib import Path
from PIL import Image
from inspection.dataset import validate
from inspection.dataset_plan import make_plan


class DatasetPlanTest(unittest.TestCase):
    def test_validation_rejects_wrong_visibility_identity_and_nan_truth(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            Image.new('RGB',(32,32),'white').save(root/'sample.png')
            plan=make_plan(['G'],per_gauge=2)[:1]
            (root/'split_plan.json').write_text(json.dumps(plan))
            (root/'collection.json').write_text(json.dumps({'outcome':'completed','restored':True}))
            row={'sample_id':plan[0]['sample_id'],'gauge_id':'G','seed':plan[0]['seed'],
                 'split':'dev','image_path':'sample.png','schema_version':1,
                 'capture':{'source':'isaac','render_count_before':0,'render_count_after':2},
                 'calibration':{'vmin':0,'vmax':10},'truth':{'value':0,'readable':True,'bbox':[0,0,32,32]}}
            def check():
                (root/'manifest.jsonl').write_text(json.dumps(row)+'\n')
                return validate(root)
            self.assertTrue(check()['valid'])
            row['truth']['readable']=False
            self.assertIn(row['sample_id']+':visibility_plan_mismatch',check()['errors'])
            row['truth']['readable']=True;row['gauge_id']='WRONG'
            self.assertIn(row['sample_id']+':condition_mismatch',check()['errors'])
            row['gauge_id']='G';row['truth']['value']=float('nan')
            self.assertIn(row['sample_id']+':invalid_numeric_label',check()['errors'])

    def test_plan_frozen_before_capture_and_disjoint(self):
        ids=[f'G{i}' for i in range(5)]
        plan=make_plan(ids)
        self.assertEqual(plan,make_plan(ids))
        self.assertEqual(len(plan),200)
        self.assertEqual(len({p['sample_id'] for p in plan}),200)
        dev=[p for p in plan if p['split']=='dev'];test=[p for p in plan if p['split']=='test']
        self.assertEqual(len(dev),100);self.assertEqual(len(test),100)
        self.assertFalse({p['seed'] for p in dev}&{p['seed'] for p in test})
        self.assertLess(max(p['distance_m'] for p in dev),min(p['distance_m'] for p in test))
        self.assertEqual({p['light_scale'] for p in test},{.8,1.15})
        self.assertEqual(sum(p['hidden'] for p in plan),10)


if __name__=='__main__':unittest.main()
