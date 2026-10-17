# Contraexemplo: retomada automática após detecção de proximidade

## Requisito relacionado

- `REQ-SAFE-AA-002`: impedir religamento automático após parada de segurança.
- `REQ-SAFE-AA-003`: permitir retomada somente após rearme explícito.

## Comportamento incorreto identificado

No supervisor preliminar, a retirada da mão após uma detecção de proximidade
podia levar diretamente do estado de parada para movimento. O comando de torque
voltava sem uma confirmação explícita do operador:

| Passo | Estado | Evento / efeito |
|---|---|---|
| 1 | `MOVING` | O scanner detecta proximidade; comando de torque vai a zero. |
| 2 | `STOP_LATCHED` | A mão sai da zona. |
| 3 | `MOVING` | O protótipo reinicia automaticamente, sem rearme do operador. |

Esse é o contraexemplo conceitual preservado por
`formal_verification/supervisor.py::counterexample_auto_resume()`. Ele ilustra
a falha do projeto preliminar; não é uma execução do supervisor corrigido.

## Correção modelada

O supervisor atual não transita diretamente de `STOP_LATCHED` para `MOVING`.
Após a parada, passa para `WAIT_CLEAR` se a zona ainda estiver ocupada ou para
`WAIT_REARM` se estiver livre. Só um evento explícito de operador em
`WAIT_REARM` permite entrar em `RESUMING`; a retomada termina em `MOVING`
apenas se a zona continuar livre. Uma nova detecção durante a retomada volta a
`STOP_LATCHED` e mantém o comando de torque desativado.

## Verificações existentes e limites

Os testes `test_no_restart_without_explicit_rearm` e
`test_proximity_during_resume_cancels_restart`, em
`tests/test_formal_supervisor.py`, exercitam esses percursos. A função
`verify_supervisor()` verifica um grafo abstrato de transições definido no
código, não executa uma exploração formal completa da implementação Python,
do firmware C ou do sistema físico. Portanto, os resultados são evidência de
testes do modelo, não uma certificação de segurança funcional.
