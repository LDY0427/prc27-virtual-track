# dash_template.html + out/dash_data.json → dash/27PRC_가상텔레메트리.html (데이터 내장, 더블클릭으로 열림)
import os, sys
here = os.path.dirname(os.path.abspath(__file__))
tpl = open(os.path.join(here, 'dash_template.html'), encoding='utf-8').read()
data_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, '..', 'out', 'dash_data.json')
data = open(data_path, encoding='utf-8').read().replace('</script', '<\\/script')
out = os.path.join(here, '27PRC_가상텔레메트리.html')
open(out, 'w', encoding='utf-8').write(tpl.replace('__DATA__', data))
print(out, os.path.getsize(out), 'bytes')
