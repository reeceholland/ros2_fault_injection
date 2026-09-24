import json
from pathlib import Path
import tempfile
import unittest
from observer_summary import render


class SummaryTests(unittest.TestCase):
    def test_missing_report_is_not_pass(self):
        self.assertIn('NO RESULT', render('/nonexistent/observer.json'))

    def test_collision_and_failures_visible_and_escaped(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder) / 'results.json'
            p.write_text(json.dumps({'passed': False, 'failures': {'no_collision': 'hit | wall'},
                'events': [{'kind': 'collision', 'time': 3.0, 'last_object': '<Wall>', 'count': 1, 'active': 1}]}))
            text = render(p)
            self.assertIn('**FAIL**', text)
            self.assertIn('hit &#124; wall', text)
            self.assertIn('&lt;Wall&gt;', text)

    def test_pass_has_no_false_collision_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder) / 'results.json'
            p.write_text(json.dumps({'passed': True, 'failures': {}, 'events': []}))
            self.assertIn('**PASS**', render(p))


if __name__ == '__main__':
    unittest.main()
