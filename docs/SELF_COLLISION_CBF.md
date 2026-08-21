# CBF cinemática de autocolisão

## Escopo desta primeira formulação

O sistema cinemático comandado pelo projeto é

\[
\dot q = u,
\]

em que `u` é publicado como velocidade articular. Para cada par não adjacente
de objetos geométricos, `k`, definimos a distância de superfície
\(d_k(q)\), a margem de segurança \(d_{safe}>0\) e a função barreira

\[
h_k(q) = d_k(q)-d_{safe}.
\]

O conjunto seguro do par é \(\mathcal C_k=\{q:h_k(q)\geq 0\}\). Usando a
função de classe K linear \(\alpha(h)=\gamma h\), com \(\gamma>0\), a CBF de
grau relativo um é

\[
\dot h_k(q,u)+\gamma h_k(q)\geq0,
\]

ou, diretamente na variável do QP,

\[
J_{d,k}(q)u\geq-\gamma\left(d_k(q)-d_{safe}\right),
\qquad J_{d,k}=\frac{\partial d_k}{\partial q}.
\]

O QP cartesiano passa a ser

\[
\begin{aligned}
\min_u\quad &\frac12\lVert J_vu-v\rVert^2
             +\frac12\lambda^2\lVert u\rVert^2\\
\text{sujeito a}\quad
             &-\dot q_{max}\leq u\leq\dot q_{max},\\
             &J_{d,k}u\geq-\gamma h_k,\quad k=1,\ldots,m.
\end{aligned}
\]

Quando \(h_k=0\), a restrição proíbe velocidade de aproximação. Quando o
estado começa fora do conjunto seguro, \(h_k<0\), ela exige uma velocidade de
separação proporcional à violação. Se as restrições geométricas e articulares
forem incompatíveis, o QP é inviável e o nó publica comando nulo; esta primeira
revisão não introduz variável de relaxação.

## Pares considerados

O backend atual usa `Robot.compute_dist_auto` do UAIbot 1.2.7. Ele compara
objetos presos a elos separados por pelo menos um elo intermediário; pares
adjacentes são excluídos, pois seus volumes se sobrepõem por construção. Todas
as linhas retornadas são mantidas, sem limiar de ativação por distância, para
não alterar silenciosamente o conjunto de segurança entre amostras.

O Xacro copia as mesmas 19 primitivas da fábrica UR3e do UAIbot: 14 cilindros,
duas esferas e três caixas. As matrizes originais, relativas aos frames DH,
foram convertidas para os frames dos elos da descrição oficial Jazzy por
transformações rígidas constantes. Assim, a cena transparente representa a
mesma geometria usada em `d_k(q)` e `J_{d,k}(q)`, dentro do arredondamento das
matrizes publicadas pelo UAIbot.

Essa equivalência não significa que a geometria seja uma reprodução fiel da
RG2 física. A fábrica UAIbot contém uma garra genérica no último elo; o Xacro
agora a exibe deliberadamente, em vez de sobrepor a antiga cápsula RG2. Portanto,
o modelo visual e o modelo matemático são coerentes entre si, mas sua fidelidade
ao conjunto UR3e/RG2 ainda deve ser avaliada separadamente.

Antes do primeiro cálculo, o adaptador verifica no wheel instalado a contagem,
o tipo, a matriz e as dimensões das 19 primitivas. Qualquer divergência em uma
versão futura do UAIbot interrompe o controlador, evitando que o Xacro e o
modelo matemático se separem silenciosamente.

Por isso o parâmetro padrão é `self_collision_cbf_mode: off`. O fluxo correto é
primeiro executar `monitor`, comparar o par e a distância mínima com a cena e
medir o custo computacional. O modo `enforce` existe para testes controlados,
mas não deve ser levado ao robô real antes da validação dimensional da geometria.

## Parâmetros iniciais

| Parâmetro | Padrão | Unidade | Função |
|---|---:|---:|---|
| `self_collision_cbf_mode` | `off` | — | `off`, `monitor` ou `enforce` |
| `self_collision_safe_distance` | `0.03` | m | distância mínima de superfície |
| `self_collision_cbf_gain` | `5.0` | s⁻¹ | taxa mínima de recuperação da barreira |
| `self_collision_distance_tolerance` | `5e-4` | m | tolerância do cálculo de distância |
| `self_collision_distance_max_iterations` | `20` | — | iterações máximas por par |

Os valores são pontos de partida experimentais, não garantias certificadas. A
taxa de controle real, o erro geométrico, a discretização e a velocidade máxima
devem entrar na escolha final da margem.

## Hipóteses e limitações

- A derivação de invariância é contínua no tempo. O controlador real é
  amostrado; a margem final deve incluir o deslocamento possível entre amostras,
  atraso de comunicação e erro do modelo.
- O Jacobiano fornecido pelo UAIbot corresponde aos pontos testemunha locais da
  distância. A função é apenas localmente/pedaço a pedaço diferenciável quando
  a característica geométrica mais próxima muda.
- A CBF é rígida. Não há folga que esconda violação; incompatibilidade entre
  separação e limites articulares resulta em parada segura.
- `monitor` mede custo e coerência, mas não demonstra invariância. `enforce`
  também não constitui certificação enquanto a geometria e a implementação
  amostrada não forem validadas.

## Referências

- A. D. Ames et al., *Control Barrier Function Based Quadratic Programs for
  Safety Critical Systems*, IEEE TAC, 2017.
  [DOI 10.1109/TAC.2016.2638961](https://doi.org/10.1109/TAC.2016.2638961).
- C. Khazoom et al., *Humanoid Self-Collision Avoidance Using Whole-Body
  Control with Control Barrier Functions*, 2022.
  [arXiv:2207.00692](https://arxiv.org/abs/2207.00692).
- UAIbot, implementação de `compute_dist_auto` usada como backend geométrico.
  [Fonte fixada no commit 1acb5ed](https://github.com/UAIbot/UAIbotPy/blob/1acb5ed637738aca4ea05945e6c065c3757bc13d/uaibot/robot/_compute_dist_auto.py).
