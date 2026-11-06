import unittest
from cv.eval import prediction_jobs,score_predictions,load_saved_predictions
import json
import tempfile
from pathlib import Path


def sample(sid,visible=True):
    return {'sample_id':sid,'gauge_id':'G','split':'test','image_path':sid+'.png',
            'calibration':{'vmin':0,'vmax':10,'unit':'bar'},'truth':{'value':5,'readable':visible,'bbox':[0,0,100,100]}}


def prediction(sid,value):
    return {'sample_id':sid,'mode':'full','prediction':{'value':value,'read_status':'ok' if value is not None else 'unreadable',
            'confidence':.9,'bbox':[0,0,100,100] if value is not None else None,'error':None if value is not None else 'covered'}}


class EvaluationTest(unittest.TestCase):
    def test_replay_rejects_different_dataset_despite_identical_ids(self):
        protocol={k:'same' for k in ('manifest_sha256','reader_sha256','settings_sha256')}
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'predictions.jsonl'
            path.write_text(json.dumps(prediction('A',5))+'\n',encoding='utf-8')
            path.with_name('protocol.json').write_text(json.dumps(protocol),encoding='utf-8')
            rows,source=load_saved_predictions(path,protocol)
            self.assertEqual(len(rows),1)
            self.assertEqual(len(source['predictions_sha256']),64)
            with self.assertRaisesRegex(ValueError,'manifest_sha256'):
                load_saved_predictions(path,{**protocol,'manifest_sha256':'another_dataset'})

    def test_predictor_input_cannot_read_true_value(self):
        class Truth(dict):
            def __getitem__(self,key):
                if key=='value':raise AssertionError('truth leaked into predictor')
                return super().__getitem__(key)
        row=sample('A');row['truth']=Truth(row['truth'])
        jobs=prediction_jobs([row],'.',('full','known_roi'))
        for job in jobs:
            self.assertNotIn('truth',job);self.assertNotIn('value',job['calibration'])
        self.assertIsNone(jobs[0]['roi']);self.assertEqual(jobs[1]['roi'],[0,0,100,100])

    def test_mae_refusals_and_confident_errors_keep_all_frames(self):
        rows=[sample('A'),sample('B'),sample('C',False)]
        metrics,details=score_predictions(rows,[prediction('A',5),prediction('B',9),prediction('C',None)])
        m=metrics['modes']['full']['test']
        self.assertEqual(m['total'],3);self.assertEqual(m['refusals'],1)
        self.assertEqual(m['mae'],2);self.assertAlmostEqual(m['normalized_mae'],.2)
        self.assertEqual(m['confident_wrong'],1);self.assertAlmostEqual(m['confident_wrong_fraction_all'],1/3)
        self.assertEqual(m['readable_success_rate'],1)

    def test_hidden_numeric_response_and_wrong_roi_are_errors(self):
        p=prediction('B',5);p['prediction']['bbox']=[200,200,100,100]
        m,_=score_predictions([sample('A',False),sample('B')],[prediction('A',5),p])
        self.assertEqual(m['modes']['full']['test']['confident_wrong'],2)
        self.assertEqual(m['modes']['full']['test']['hidden_false_reads'],1)


if __name__=='__main__':unittest.main()
