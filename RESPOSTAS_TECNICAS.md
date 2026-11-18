# DOG-IA V2 — Respostas às Questões Técnicas (AC-4)

---

## Questão 1 — Arquitetura Híbrida de Navegação (Global vs. Local)

**Qual limitação crítica da navegação puramente reativa é resolvida pela arquitetura híbrida, e por que o planejador global sozinho também não basta?**

A navegação puramente reativa (V1) fica presa em **mínimos locais**: quando o robô encontra um obstáculo em forma de "U" (por exemplo, uma banca encostada numa parede), os raios do LiDAR bloqueiam todos os ângulos frontais e o robô oscila indefinidamente sem encontrar saída, pois ele só enxerga o que está imediatamente à frente.

A arquitetura híbrida resolve isso ao separar duas camadas:
- **Planejador global (A\*)** calcula antecipadamente uma rota livre de obstáculos conhecidos usando a grade de ocupação com inflação (C-Space), garantindo que o caminho contorne qualquer mínimo local.
- **Evasão reativa (LiDAR)** lida com o que o planejador não previu: obstáculos dinâmicos (pedestres, veículos infratores), objetos recém-adicionados e imprecisões do sensor.

O planejador global sozinho **não basta** porque:
1. O ambiente urbano é dinâmico — carros, pedestres e ciclistas surgem em tempo real e não estão na grade.
2. O A* opera sobre uma grade discreta de 10×10 px; objetos entre células ou em movimento não são representados.
3. Erros de odometria e ruído do LiDAR fazem o robô desviar da trajetória planejada, exigindo correção local contínua.

---

## Questão 2 — Máquina de Estados

### 2.1 — Estados e transições em uma viagem completa (destino 1: Mercado)

| Ordem | Estado              | Condição que causou a transição                                      |
|-------|---------------------|----------------------------------------------------------------------|
| 1     | `NAVEGANDO`         | Estado inicial; A* fornece waypoints e o LiDAR não detecta perigo.  |
| 2     | `AGUARDANDO_SINAL`  | Robô alcança proximidade do meio-fio; semáforo **não** está em PEDESTRE. |
| 3     | `ATRAVESSANDO`      | Semáforo entra na fase PEDESTRE **e** `tempo_restante > t_min`.     |
| 4     | `NAVEGANDO`         | Robô cruzou a rua; calçada inferior, sem obstáculos frontais.       |
| 5     | `DESTINO_ALCANCADO` | `dist_alvo ≤ 28 px` — robô chegou ao Mercado.                      |

> **Observação:** se um veículo infrator cruzar a rota durante a travessia, o estado passa por `FREIO_EMERGENCIA` antes de voltar a `ATRAVESSANDO`.

### 2.2 — Diferença entre `AGUARDANDO_SINAL` e `FREIO_EMERGENCIA`

| Aspecto          | `AGUARDANDO_SINAL`                                      | `FREIO_EMERGENCIA`                                       |
|------------------|---------------------------------------------------------|----------------------------------------------------------|
| Causa            | Semáforo não autoriza travessia (fase incorreta ou tempo insuficiente). | Swept-volume de um veículo intercepta o robô ou o tutor. |
| Natureza         | **Planejada** — o robô sabe com antecedência que deve esperar. | **Reativa** — acionada em tempo real pelo sensor.        |
| Duração          | Determinística: dura até a próxima fase PEDESTRE.       | Dura enquanto a zona de perigo permanecer ativa.         |
| Velocidade       | Robô para suavemente ao lado do meio-fio.               | Parada imediata (vel = 0).                               |
| Contador métrico | Não incrementa `freios_emergencia`.                     | Incrementa `freios_emergencia` e `alertas_vermelhos`.    |

---

## Questão 3 — Percepção e Ruído do Sensor

### 3.1 — Por que mediana em vez de média?

As falhas do tipo dropout retornam **alcance máximo** (165 px), que é um valor extremo e raro. A **média aritmética** é puxada fortemente por esses outliers: 2 leituras normais de 40 px + 1 falha de 165 px resultam em média de ~82 px — uma distância totalmente errada que poderia fazer o robô frear desnecessariamente ou avançar sobre um obstáculo.

A **mediana** seleciona o valor central da janela de 3 amostras ordenadas; um único outlier extremo (165 px) é descartado, e as duas leituras reais dominam. Para qualquer número ímpar de amostras com até `(n-1)/2` outliers, a mediana é **robusta** e preserva a leitura real.

### 3.2 — Custo do filtro em termos de atraso

O filtro de mediana com janela de **3 quadros** espera acumular 3 leituras antes de produzir um valor estável. Com 60 varreduras/segundo:

```
atraso = (3 - 1) / 60 = 2 / 60 ≈ 0,033 s  (~33 ms)
```

Ou seja, um obstáculo que aparece do nada levará **~33 ms** para ser confirmado pelo filtro. A 2,5 px/quadro (~150 px/s), o robô avança ~5 px nesse intervalo — completamente seguro para as distâncias de reação de 65–85 px usadas no código.

### 3.3 — O que aconteceria com janela de 15 varreduras?

```
atraso = (15 - 1) / 60 = 14 / 60 ≈ 0,233 s  (~233 ms)
```

Com 150 px/s, o robô avança **~35 px** antes de confirmar o obstáculo. Como o limiar de frenagem é 85 px, o robô só "veria" o obstáculo a ~50 px, sem margem para desvio suave — aumentaria drasticamente colisões e freios de emergência. Além disso, 15 amostras criam um efeito de "suavização excessiva" que mascara variações reais do ambiente (veículos passando, obstáculos removidos).

---

## Questão 4 — Segurança Temporal na Travessia

### 4.1 — Cálculo do limite mínimo

Fórmula: `t_min = (RUA_ALTURA + 24 + GUIA) / velocidade × 1,35 + 0,4`

Substituindo os valores:
- `RUA_ALTURA = 170 px`
- `folga = 24 px`
- `GUIA = 42 px`
- `velocidade = 2,5 px/frame × 60 fps = 150 px/s` *(o enunciado usa 156, usando esse valor abaixo)*

**Com 156 px/s (valor do enunciado):**
```
t_min = (170 + 24 + 42) / 156 × 1,35 + 0,4
t_min = 236 / 156 × 1,35 + 0,4
t_min = 1,513 × 1,35 + 0,4
t_min = 2,042 + 0,4
t_min ≈ 2,44 s
```

Portanto, o robô só inicia a travessia se restar **pelo menos ~2,44 segundos** na fase PEDESTRE.

### 4.2 — Por que fases "todos vermelhos" de 1,5 s?

As fases `TODOS_VERM_1` e `TODOS_VERM_2` são **intervalos de segurança (clearance intervals)**. Elas eliminam o risco de **colisão por sobreposição de fases**: um veículo que cruzou o sinal no último instante do VERDE ainda estará atravessando a faixa quando o sinal do pedestre acenderia imediatamente. Os 1,5 s garantem que:

1. Todos os veículos em movimento completem a travessia da via antes que pedestres entrem.
2. Veículos infratores que "colaram" no AMARELO sejam absorvidos antes da liberação de pedestres.
3. O sistema tolera latência humana e de freio (~0,8–1,2 s de reação + distância de frenagem).

---

## Questão 5 — Avaliação Experimental

### 5.1 — Resultados `--benchmark 20`

Execute com os comandos abaixo e preencha a tabela com os valores impressos no terminal:

```bash
python dog_ia_v2.py --benchmark 20 --seed 1
python dog_ia_v2.py --benchmark 20 --seed 7
```

| Métrica              | seed=1   | seed=7   |
|----------------------|----------|----------|
| Taxa de sucesso      | __%      | __%      |
| Tempo médio (s)      | __s      | __s      |
| Freios emergência    | __       | __       |
| Alertas vermelhos    | __       | __       |

*(Os valores dependem do hardware; preencher após execução.)*

### 5.2 — Por que seed fixa é reproduzível, mas seeds diferentes divergem?

A **seed fixa** inicializa o gerador pseudoaleatório (Python `random` + NumPy) em um estado determinístico, garantindo que toda posição inicial de veículo, ruído gaussiano e dropout sigam exatamente a mesma sequência. Isso torna o experimento **reproduzível**: rodar `--seed 1` duas vezes produz resultados idênticos.

Mesmo assim, **seeds diferentes divergem** porque a sequência de números pseudoaleatórios muda completamente com outro valor inicial — os veículos partem de posições diferentes, os infratores surgem em momentos distintos, e o ruído do LiDAR segue outro padrão. Pequenas diferenças iniciais se amplificam (efeito borboleta) no ambiente dinâmico, levando a trajetórias, tempos e colisões completamente distintos.

### 5.3 — Melhoria concreta proposta

**Proposta:** aumentar o custo A* na faixa de pedestres durante fase não-PEDESTRE de 1.0 para 3.5, penalizando rotas que atravessam a rua em momento proibido.

| Métrica esperada      | Efeito         | Justificativa                                                    |
|-----------------------|----------------|------------------------------------------------------------------|
| Freios de emergência  | **Diminui**    | Robô evita entrar na rua em tempo perigoso ainda no planejamento. |
| Tempo médio de viagem | **Aumenta**    | O planejador encontra rotas mais longas para evitar a travessia proibida. |
| Taxa de sucesso       | **Aumenta**    | Menos colisões = mais viagens concluídas com segurança.          |

---

*DOG-IA V2 — AC-4 | Simulador de Cão-Guia Autônomo Assistivo*
