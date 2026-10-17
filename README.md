# Operação Linha Viva — Braço Colaborativo AuroraArm (Junta do Cotovelo)

Este repositório contém o pacote completo de Model-Based Design (MBD), verificação formal de segurança, geração de código C rastreável e suíte de testes Software-in-the-Loop (SIL) para a junta do cotovelo do robô colaborativo **AuroraArm**

---

## Visão Geral e Parâmetros da Junta

A junta do cotovelo é acionada por um motor CC de ímã permanente de 48 V com transmissão harmônica de redução 100:1.

- **Estado Mínimo da Planta:** $x = [i, \omega]^T$ (Corrente na armadura e velocidade angular do motor).
- **Integração de Posição:** Posição angular da junta $\theta_{\text{junta}} = \theta_{\text{motor}} / 100$.
- **Saturação de Atuação:** $\pm 48\text{ V}$ máximo e limite de corrente do driver de $\pm 8\text{ A}$.
- **Período de Amostragem ($T_s$):** $2\text{ ms}$ ($500\text{ Hz}$), com orçamento de tempo de execução (WCET) $< 200\,\mu\text{s}$ no microcontrolador ARM Cortex-M4.
- **Ponto Operacional Crítico:** $\omega_{ss} = 450\text{ rad/s}$ sob torque refletido de $0{,}25\text{ N}\cdot\text{m}$ ($i_{ss} \approx 3{,}735\text{ A}$, $V_{ss} \approx 41{,}24\text{ V}$, folga de $6{,}76\text{ V}$ / $14{,}1\%$).

---

##  Orçamento de Tempo de Segurança e Parada Física (`REQ-SAFE-AA-001`)

O prazo máximo de $100\text{ ms}$ para remoção de torque e parada é garantido pela composição determinística dos atrasos digitais e do mecanismo de segurança Safe Torque Off (STO):

1. **Atraso Máximo do Sensor de Proximidade:** $20{,}0\text{ ms}$
2. **Jitter / Perda de Ciclo RTOS:** $2{,}0\text{ ms}$
3. **Espera pela Próxima Varredura da Malha ($T_s$):** $2{,}0\text{ ms}$
4. **Execução do Laço (WCET):** $0{,}2\text{ ms}$ (Limite por ciclo no ARM Cortex-M4)
5. **Subtotal do Orçamento Digital:** $24{,}2\text{ ms}$
6. **Tempo de Atuação Física (STO SIL 3 e Freio Eletromagnético):**
   - Desconexão elétrica da ponte H via STO: $2{,}0\text{ ms}$
   - Engate mecânico do freio: $25{,}0\text{ ms}$
   - Decaimento de velocidade até corrente residual $< 0{,}05\text{ A}$ (Torque zerado): $45{,}0\text{ ms}$
   - Subtotal da Parada Física: $72{,}0\text{ ms}$
7. **Tempo Total no Pior Caso:** $\mathbf{24{,}2\text{ ms} + 72{,}0\text{ ms} = 96{,}2\text{ ms} \le 100{,}0\text{ ms}}$ (Atende ao requisito com margem de $3{,}8\text{ ms}$).

---

##  Matriz de Rastreabilidade Técnica Completa

A matriz abaixo estabelece o mapeamento bidirecional direto entre os requisitos do enunciado, os módulos de simulação/modelo, o firmware C gerado e a suíte de testes de validação:

| ID Requisito | Descrição | Módulo do Modelo | Código C / Firmware | Suíte de Testes / Validação | Status |
| :--- | :--- | :--- | :--- | :--- | :---: |
| **REQ-CTRL-AA-001** | Erro de posição em regime permanente nulo para referência em degrau. | `model/plant.py` | `generated_code/aurora_arm_control.c` | `tests/test_model_and_controller.py::test_position_step_performance_and_actuator_limits` | **Aprovado** |
| **REQ-CTRL-AA-002** | Sobressinal da resposta de posição $\le 8\%$. | `controller/reference.py` | `generated_code/aurora_arm_control.c` | `tests/test_model_and_controller.py::test_position_step_performance_and_actuator_limits` | **Aprovado** |
| **REQ-CTRL-AA-003** | Tempo de acomodação $\le 300\text{ ms}$ (banda de $\pm 2\%$). | `controller/reference.py` | `generated_code/aurora_arm_control.c` | `tests/test_model_and_controller.py::test_position_step_performance_and_actuator_limits` | **Aprovado** |
| **REQ-CTRL-AA-004** | Tensão de comando $\le 48\text{ V}$ e Corrente $\le 8\text{ A}$ em todos os instantes. | `model/plant.py` / `controller/reference.py` | `generated_code/aurora_arm_control.c` | `tests/test_model_and_controller.py::test_position_step_performance_and_actuator_limits` | **Aprovado** |
| **REQ-SAFE-AA-001** | Torque da junta zerado em no máximo $100\text{ ms}$ após detecção do escâner. | `formal_verification/supervisor.py` | `generated_code/aurora_arm_safety.c` | `tests/test_formal_supervisor.py::test_all_sensor_jitter_phase_combinations_meet_command_deadline` | **Aprovado** |
| **REQ-SAFE-AA-002** | Proibição de retomada automática de movimento após interrupção. | `formal_verification/supervisor.py` | `generated_code/aurora_arm_safety.c` | `tests/test_formal_supervisor.py::test_no_restart_without_explicit_rearm` | **Aprovado** |
| **REQ-SAFE-AA-003** | Retomada da trajetória original somente após rearme explícito do operador. | `formal_verification/supervisor.py` | `generated_code/aurora_arm_safety.c` | `tests/test_proximity_during_resume_cancels_restart` | **Aprovado** |
| **REQ-SAFE-AA-004** | Velocidade linear do efetuador $\le 250\text{ mm/s}$ na zona colaborativa. | `model/plant.py` / `controller/reference.py` | `generated_code/aurora_arm_control.c` | `tests/test_model_and_controller.py::test_position_step_performance_and_actuator_limits` | **Aprovado** |

---

## 🚀 Como Executar e Reproduzir

### Pré-requisitos
- Python 3.10+
- Compilador C instalado (`gcc`, `clang`, `cl.exe` ou `zig`)

### Execução dos Testes e Geração de Código
```powershell
# 1. Gerar o código C do controlador a partir do modelo de referência
py generated_code\generate_c.py

# 2. Executar o teste Software-in-the-Loop (requer compilador C)
py generated_code\run_sil.py

# 3. Executar a suíte de testes automatizados e verificação formal
py -m unittest discover -s tests -v