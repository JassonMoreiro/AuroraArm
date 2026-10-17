# Evidências de hardware

Os valores desta nota foram enviados pelo estudante em 17/10/2026. Não foram
medidos nem certificados por este repositório. Antes de usá-los numa auditoria,
anexar certificados, datasheets, identificadores de equipamento, firmware,
relatórios de ensaio e logs de medição.

| Evidência relatada | Valor declarado | Situação documental |
|---|---:|---|
| Unidade MCU/FPU / relógio | Cortex-M4, 168 MHz | Informado pelo estudante; modelo exato e configuração não anexados |
| Método WCET | DWT `CYCCNT` | Método informado; captura bruta/configuração não anexada |
| WCET medido | 42,8 µs por ciclo de 2 ms | Informado pelo estudante; log e método de pior caso não anexados |
| Orçamento | <200 µs/ciclo | Requisito do enunciado |
| Margem de CPU | (200−42,8)/200=78,6% do orçamento | Cálculo derivado do WCET declarado |
| Uso do ciclo de 2 ms | 42,8/2000=2,14%; 97,86% do ciclo restante | Denominador diferente da margem do orçamento WCET |
| Binário | 38,4 KB, GCC `-O3 -mfloat-abi=hard` | Informado; ELF/map e condições de build não anexados |
| Flash disponível | 256 KB | Requisito do enunciado |
| Ocupação | 38,4/256=15,0% | Cálculo derivado do tamanho declarado |
| HIL | STM32F4 Discovery, comunicação CAN/UART a 500 Hz | Informado; roteiro, logs e configuração não anexados |
| STO SIL 3 / PL e | Conforme IEC 61800-5-2 / IEC 61508 | Alegação informada; certificado e identificação do componente não anexados |
| Tempo de interrupção de corrente pelo STO | 2 ms | Informado; oscilograma/critério de limiar não anexados |
| Engate do freio | 25 ms | Informado; datasheet/relatório não anexado |
| Frenagem até parada total | 45 ms | Informado; carga, velocidade e tolerância de ensaio não anexadas |
| Total de parada mecânica/elétrica | 72 ms | Soma informada; condições de validade não anexadas |
| Corrente residual | `<0,05 A` equivale a torque motor `<0,00425 N·m` | Critério proposto; incerteza do sensor e medição de bancada não anexadas |

## Interpretação conservadora do resultado temporal

`formal_verification/supervisor.py` enumera 126 combinações: sensor 0–20 ms,
ciclo perdido 0 ou 2 ms, fase de amostragem 0–2 ms, e até 0,2 ms de execução
(cálculo conservador para requisito estrito `<200 µs`). O pior limite superior
digital é 24,2 ms. Incluindo os 72 ms de parada total declarados pelo estudante:

`20 + 2 + 2 + 0,2 + (2 + 25 + 45) = 96,2 ms <= 100 ms`

A margem aritmética é 3,8 ms, condicionada a todos os limites informados serem
comprovados e válidos simultaneamente nas condições máximas de velocidade,
carga, temperatura, tensão e desgaste. O limiar de corrente `<0,05 A` e o
nível lógico STO baixo são evidências distintas: sinalizar STO baixo comprova
pedido de desligamento, não comprova corrente residual abaixo do limiar. Para
REQ-SAFE-AA-001, o critério físico precisa ser instrumentado e medido. O
indicador de aprovação aritmética nos resultados não significa certificação
independente da resposta física.

## Referência do caso de integração fornecida

Foi verificado o URL direto `https://www.universal-robots.com/case-stories/vallurbana/`
em 17/10/2026; o servidor respondeu HTTP 404. O endereço citado pelo estudante
na forma `https://www.google.com/search?q=...` é uma pesquisa do Google, não o
relato primário. Portanto, a referência Vallurbana proposta não está validada
e não deve ser apresentada como fonte verificada até ser fornecido um URL
primário funcional ou cópia/publicação identificável.
