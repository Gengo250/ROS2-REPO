# Modelagem procedural do AMR

Fonte de verdade: `../../config/amr_dimensions.json` e estes scripts, nunca edições manuais do `.blend`.
Convenção: **X forward, Y left, Z up; meters**. Escala de objeto `1, 1, 1`.

Na raiz do pacote:

```bash
# Cena + validação + oito meshes + .blend + cinco renders:
blender --background --factory-startup --python-exit-code 1 --python scripts/blender/generate_amr.py
# Sem renders:
blender --background --factory-startup --python-exit-code 1 --python scripts/blender/generate_amr.py -- --no-renders
# Exportação independente:
blender --background blender/warehouse_amr.blend --python-exit-code 1 --python scripts/blender/export_amr.py
# Validação independente:
blender --background blender/warehouse_amr.blend --python-exit-code 1 --python scripts/blender/validate_amr.py
```

O gerador usa uma cena própria (`AMR_Generation`) e Collections `AMR/CHASSIS`, `PLATFORM`,
`DRIVE`, `CASTERS`, `SENSORS`, `DEBUG`. As duas rodas compartilham a mesma malha.
Somente a roda esquerda e o apoio frontal são exportados; o Xacro reutiliza essas geometrias.
STLs têm origem local ao respectivo link, nunca a transformação de montagem completa.

Blender local validado: **5.2.2 LTS**, Python interno **3.13.13**. O script não usa pacotes
Python externos ao Blender e à biblioteca padrão.
Nenhum Blender MCP local foi encontrado/configurado nesta sessão. Existe capacidade remota
de Blender no MCP Higgsfield, mas seus artefatos publicados são `.blend`/GLB/imagens, sem
acesso direto a este workspace nem exportação de STLs por peça. Por isso, a execução de
produção e os testes usaram o Blender local headless, conforme o fallback solicitado.

Se um MCP local que execute Python for conectado, envie **o carregamento do arquivo salvo**:

```python
import importlib.util
from pathlib import Path
path = Path('/home/gengo/ros2_ws/src/amr_description/scripts/blender/generate_amr.py')
spec = importlib.util.spec_from_file_location('amr_generator', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.main()  # Mesma geração, validação, exportação e renders do modo headless.
```

Não recrie objetos por comandos soltos do MCP. Correções devem ser salvas no script/JSON
e regeneradas. Nenhuma seleção manual, clique, reconhecimento visual ou macro é necessária.
O gerador recusa sobrescrever uma cena homônima que não esteja marcada como sua.

Para alterar dimensões, validar Xacro, abrir RViz ou Gazebo, consulte o
[README do pacote](../../README.md). Para todo o fluxo: `bash scripts/build_amr.sh`.
