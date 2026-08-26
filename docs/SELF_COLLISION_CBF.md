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

O backend do projeto percorre objetos presos a elos separados por pelo menos um
elo intermediário; pares adjacentes são excluídos, pois seus volumes se
sobrepõem por construção. Para cada par, usa `UAIbot.Utils.compute_dist` em modo
Python e monta o Jacobiano da distância a partir dos pontos testemunha e dos
Jacobianos DH. Todos os pares são mantidos, sem limiar de ativação, para não
alterar silenciosamente o conjunto de segurança entre amostras.

Esse avaliador substitui somente o invólucro `Robot.compute_dist_auto` do UAIbot
1.2.7. Nessa revisão, `Utils.compute_dist` documenta e retorna quatro valores,
mas `_compute_dist_auto_python` tenta desempacotar três. O projeto trata
explicitamente os quatro valores e mantém o wheel instalado sem modificações.

O projeto valida inicialmente as 19 primitivas da fábrica UR3e do UAIbot 1.2.7.
Em seguida, substitui essa lista por um modelo corrigido com 16 objetos. As 13
primitivas do braço preservam os tipos da fábrica; seis recebem ajustes
versionados de pose ou dimensão para cobrir as juntas físicas. Os seis objetos
da garra genérica são removidos e substituídos por uma cápsula RG2 formada pela
união de um cilindro e duas esferas.

As matrizes do braço, relativas aos frames DH, foram convertidas para os frames
dos elos da descrição oficial Jazzy por transformações rígidas constantes. A
cópia original posicionava os objetos `c21` e `c22` em `z=0,050 m` no
`forearm_link`; o modelo do projeto usa `z=0,027 m`, igual ao `elbow_offset`
publicado para o UR3e. A esfera `c21` é centralizada na junta e usa o
`elbow_radius=0,060 m`. O cilindro `c23` cobre os `0,10405 m` entre esse offset
e `wrist_1_link`; `c31` é centralizado no primeiro punho e `c32` cobre os
`0,08535 m` completos até `wrist_2_link`. Por fim, `c41` é centralizado no
segundo punho. Os raios de `c23` e `c41` mantêm uma folga positiva entre os
elos não adjacentes 2 e 4, evitando uma autocolisão estrutural falsa. Tudo é
aplicado simultaneamente à tabela matemática e ao Xacro. A
cápsula usa raio de `0,090 m`, trecho cilíndrico de `0,110 m` e centros das
extremidades em `z=0,055 m` e `z=0,165 m` no frame da RG2. Esses mesmos valores
são anexados ao último elo do modelo UAIbot. Assim, a cena transparente e os
cálculos de `d_k(q)` e `J_{d,k}(q)` usam a mesma geometria corrigida.

O código da dependência instalada não é alterado. A substituição ocorre após
`Robot.create_ur_ur3e()` por meio dos objetos de cada `Link`. Antes e depois da
troca, o adaptador verifica contagem, tipo, matriz e dimensões; qualquer
divergência interrompe o controlador. A CBF corrigida está habilitada somente
para o conjunto `ur3e + rg2`; outros modelos são recusados em `monitor` e
`enforce` até possuírem geometria própria.

### Ajuste dos volumes do braço

A tabela `UR3E_UAIBOT_PRIMITIVES` é o contrato imutável da dependência fixada.
Os ajustes experimentais devem ser feitos somente em
`UR3E_RG2_PROJECT_PRIMITIVES`, no módulo
`ur_cbf_control/uaibot_collision_model.py`. Cada uma das 13 primitivas do braço
é uma cópia independente, permitindo alterar matriz e dimensões sem modificar a
referência usada para validar o wheel. O Xacro deve receber o mesmo ajuste; os
testes de regressão verificam origens, orientações, tipos e dimensões antes da
publicação.

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
- UAIbot, implementação de `compute_dist_auto` cuja incompatibilidade de
  desempacotamento é contornada pelo projeto.
  [Fonte fixada no commit 1acb5ed](https://github.com/UAIbot/UAIbotPy/blob/1acb5ed637738aca4ea05945e6c065c3757bc13d/uaibot/robot/_compute_dist_auto.py).
- UAIbot, implementação pública de `Utils.compute_dist` usada para as distâncias
  diferenciáveis.
  [Fonte fixada no commit 1acb5ed](https://github.com/UAIbot/UAIbotPy/blob/1acb5ed637738aca4ea05945e6c065c3757bc13d/uaibot/utils/utils.py).
- Universal Robots, parâmetros físicos oficiais do UR3e, incluindo
  `elbow_offset: 0.027`.
  [Descrição ROS 2 Jazzy, commit 3924298](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/39242984dc8d1fff9584c922c17c69c58df3591d/config/ur3e/physical_parameters.yaml).
