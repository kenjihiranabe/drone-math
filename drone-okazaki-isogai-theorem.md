<!-- GitHubとKaTeXの両方で \bm を有効にするマクロ定義 -->
$$ \gdef\bm[#1]{\boldsymbol{#1}} $$

# 論文「飛行形ドローンのモータ故障問題」数式ウォークスルー

## これは何か？
ドローンのモーターが故障した場合、他のモーターのみで安全に飛行するように制御することはできるだろうか？という問いに、この論文はドローンの運動を剛体としてモデル化し、モーター故障時の制御可能性を定式化しています。

しかし、論文中の数式は導出プロセスの詳細が記述されていない部分もあります。本ドキュメントでは、論文に登場する主要な数式について、その物理的意味や導出プロセス、座標変換の流れなどを順を追って詳しく解説します。

- 対象文献「飛行形ドローンのモータ故障問題」：岡崎秀晃、磯貝海斗
https://www.jstage.jst.go.jp/article/essfr/14/1/14_44/_pdf/-char/ja

- 参考文献「ドローンの物理シミュレータの構築」：箱庭physics
https://github.com/toppers/hakoniwa-px4sim/blob/main/drone_physics/README-ja.md

## 本ドキュメントに含まれる、論文以外の情報

>[!NOTE]
>1. 岡崎・磯貝の定理（式 32）を導く過程の、数学（特に回転とその時間微分の扱い）を詳しく解説しました。
>2. 上記定理の「一般化座標＋仮想仕事の原理＋静止座標系」を基準にした導出を、より単純な「ニュートン・オイラーの運動方程式＋動座標系」を基準にした導出のみで説明してみました。 $\bm{\omega}_{\dot{\bm{x}}}\rightarrow \bm{\Omega}_{\dot{\bm{x}}}$ 
>3. これによって、オイラー角のうちヨー角（$\psi$）のみを分離できることもはっきり分かりました。$\bm{\Omega}_{\dot{\bm{x}}} = \bm{\Omega}_{\dot{\bm{x}}}(\theta, \phi)$
>4. その過程で、定理の式を具体的な行列成分として書き出した。

---

## 3.1節：マルチロータの運動の数学的基礎（式 1〜8）

第3.1節では、ドローンの位置と姿勢を表現するための「静止（慣性）座標系」「動（機体）座標系」の定義、および座標変換のための数式を定義します。ここでは、理論的な基礎の準備をしています。さっと流していきましょう。

### ■ アフィン空間における作用と剛体の運動の定義（式 1, 2, 3）
$$a \mapsto a + \bm{v}, \quad a \in A^3, \; \bm{v} \in V \quad \text{--- (式 1)}$$
* **意味:** 3次元空間の幾何学的な「点」( $a$ )の集合であるアフィン空間 $A^3$ に対し、位置のズレ（相対ベクトル） $\bm{v}$ を表すベクトル空間 $V$ が平行移動として作用することを示しています。
* **物理的な位置づけ:** ドローンの移動を、点とベクトルの加算によって数学的に厳密に定義しています。
  * 静止座標系（慣性軸系）: $O + \text{span}\{\bm{e}_1, \bm{e}_2, \bm{e}_3\}$
  * 動座標系（質量中心 $O_c$ に固定）: $O_c + \text{span}\{\bm{E}_1, \bm{E}_2, \bm{E}_3\}$

この座標系において、剛体運動を定義します。並行移動と回転移動の組み合わせです。

$$\bm{T}(t): W \to w \quad \text{--- (式 2)}$$
$$\bm{T}(t) = \bm{C}(t)\bm{B}(t) \quad \text{--- (式 3)}$$

式(2)は、剛体（ドローン機体）の運動 $\bm{T}(t)$ は、動座標系 $W$ から静止座標系 $w$ へのアフィン写像として定義されます（写像の「定義域＝入力の集合 $W$ 」と「終域＝出力の集合 $w$ 」）。変換の向きに注意してください。動座標系のベクトルを静止座標系へと変換します。本論文では、一貫して動座標系→静止座標系が順方向です。この方が変換行列の表記として自然になります。

式(3)は、この運動が **「回転行列 $\bm{B}(t)$」** と **「並進（位置の移動）$\bm{C}(t)$」** の合成として一意に表せることを示しています。ここで、 $\bm{B}(t)\bm{C}(t)$ は、あたかも掛け算のように書かれていますが、実際には掛け算ではなく、「回転行列 $\bm{B}(t)$ による座標変換」と「並進ベクトル $\bm{C}(t)$ による位置の移動」の
**合成変換**（すなわちアフィン変換）を表しています。また、両者が時間 $t$ の関数になっていることにも注意してください。

### ■ 動径ベクトルと速度ベクトル（式 4, 5）

これを具体的な位置ベクトルと速度ベクトルの式に展開していきます。なお、一貫して小文字で静止座標系のベクトル（ $w$ の元）、大文字で動座標系のベクトル（ $W$ の元）を表す慣習が続きます。


$$\bm{q}(t) = \bm{r}(t) + \bm{B}(t)\bm{Q}(t) \quad \text{--- (式 4)}$$

$$
\dot{\bm{q}} = \dot{\bm{r}} + \frac{d}{dt}(\bm{B}\bm{Q}) = \dot{\bm{r}} + \dot{\bm{B}}\bm{Q} + \bm{B}\dot{\bm{Q}}\quad \text{--- (式 5)}
$$

式(4, 5)で、$\bm{q}, \bm{r} \in w$ は、静止座標系における位置ベクトルであり、$\bm{Q} \in W$ は、動座標系における位置ベクトル（動径ベクトルと呼んでいる）を表しています。

* **式(4)の意味:** 静止座標系から見た動点の位置 $\bm{q}(t) \in w$ は、「重心の位置 $\bm{r}(t) \in w$」に「機体内の相対位置 $\bm{Q}(t) \in W$ を回転行列 $\bm{B}(t)$ で静止座標系へ変換したベクトル $\bm{B}(t)\bm{Q}(t) \in w$ 」を加えたものです。
* **式(5)の意味:** これを時間微分した速度式です。機体が変形しない剛体であれば、機体内での相対位置の変化はないため、第3項 $\bm{B}\dot{\bm{Q}} = \bm{0}$ となります。

この(式4,5)から、非常に汎用的な運動方程式(式13)が後に導出されます。さらに準備として、ドローンの姿勢を表すテイト-ブライアン角（ヨー、ピッチ、ロール）に対応する基本回転行列を定義します。

### ■ テイト-ブライアン角に対応する基本回転行列（式 6, 7, 8, 9）

ドローンの姿勢（回転）は、ヨー（$\psi$）、ピッチ（$\theta$）、ロール（$\phi$）の3つの軸まわりの回転を、この順に静止座標系の基底を回転して動座標系の基底に重ねることで表現します（テイト・ブライアン角）。3つの角の順番にはこの定義以外様々な流儀（12種類）があるので注意してください。論文での定義は以下の通りです：

| ヨー回転 $\bm{R}_{\psi}$<br>（Z軸まわり、式 6） | ピッチ回転 $\bm{R}_{\theta}$<br>（中間Y軸まわり、式 7） | ロール回転 $\bm{R}_{\phi}$<br>（機体X軸まわり、式 8） |
| :---: | :---: | :---: |
| $\bm{R}_{\psi} = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}$ | $\bm{R}_{\theta} = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}$ | $\bm{R}_{\phi} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix}$ |

注：これは箱庭physicsで採用されているオイラー角と同じ順番、同じ記号の右手形です。ただし箱庭はz軸が下向きである点が異なります。
箱庭physics(https://github.com/toppers/hakoniwa-px4sim/blob/main/drone_physics/README-ja.md)では、
- 機体座標系 body frame は FRD(front-right-down)。
- 地上座標系 ground frame は NED(north-east-down)。

理論的には重力方向の扱いと制御時のラダー・エレベータ・エルロンの符号が異なりますが、以下の論文と基本的には箱庭physicsとそのまま同じ式が成り立ちます。

また、ドローンの姿勢を表す回転行列 $\bm{B}$ は、これらの基本回転行列を掛け合わせることで構成されます（式 9）。この合成回転行列 $\bm{B}$ を用いて、機体固定座標系から静止座標系への変換を行います。

>[!IMPORTANT]
>$$\bm{B}(\psi, \theta, \phi) = \bm{R}_{\psi}\bm{R}_{\theta}\bm{R}_{\phi} \quad \text{--- (式 9)}$$

回転行列 $\bm{B}$ について、少し詳しくまとめておきます（以下は飛ばしても大丈夫）。

> [!NOTE]
> **回転変換行列 $B$**
>上記箱庭physicsに計算が書かれていますが、このように３つの回転をこの順にかけることで、静止座標系から動座標系へと基底の取り替え行列としての $\bm{B}$ が定義されます。
>$$
>\begin{bmatrix}
>\bm{E}_1 & \bm{E}_2 & \bm{E}_3
>\end{bmatrix}
>=\begin{bmatrix}
>\bm{e}_1 & \bm{e}_2 & \bm{e}_3
>\end{bmatrix}
>\bm{B}
>$$
>
>このように見ると、例えば $\bm{B}$ の一列目は $\bm{E}_1$ を静止座標系で表現したときの成分表示になっていることが見て取れるでしょう。
>
>さらに、ベクトルの普遍性（「基底」と「座標」の掛け算が変化しない）より、
>$$
>\begin{bmatrix}
>\bm{e}_1 & \bm{e}_2 & \bm{e}_3
>\end{bmatrix}
>\begin{bmatrix}
>x \\ y \\ z
>\end{bmatrix}
>=\begin{bmatrix}
>\bm{E}_1 & \bm{E}_2 & \bm{E}_3
>\end{bmatrix}
>\begin{bmatrix}
>x' \\ y' \\ z'
>\end{bmatrix}
>$$
>よって、それぞれの基底を使った座標表現の変換は以下のようになります。
>$$
>\begin{bmatrix}x \\ y \\ z\end{bmatrix}
>= \bm{B}
>\begin{bmatrix}x' \\ y' \\ z'\end{bmatrix} \quad \text{すなわち} \quad
>\begin{bmatrix}
>静\\止\\座\\標
>\end{bmatrix}
>= \bm{B}
>\begin{bmatrix}
>動\\座\\標
>\end{bmatrix}
>$$
>これは以降でもとてもよく使われる、静止座標と動座標の変換公式です。どんなベクトル $a$ についてもこれが成り立ちます。例えば速度ベクトル、各速度ベクトル、力ベクトル、トルクベクトルなどです。
>
>いくつか覚えておくべき性質を上げます。
>- $\bm{B}$ は直交行列である。すなわち、$\bm{B}\bm{B}^T=\bm{B}^T\bm{B}=I$ (単位行列)
>- $\bm{B}$ はある１つの軸周りの回転として定義できる。その軸は $\bm{B}$ の固有値1に対応する固有ベクトル（動座標系）であるこのベクトルは、変換の前後で変化しない。
>- 任意の動座標ベクトル $\bm{A}$ に対して、静止座標系での表現は $\bm{a} = \bm{B}\bm{A}$ となる。逆に、静止座標系のベクトル $\bm{a}$ を動座標系で表すときは $\bm{A} = \bm{B}^T\bm{a}$ となる。
>- $\bm{B}$ は回転の構造を保つ。すなわち、ベクトルの外積をとってから回転しても、ベクトルをあらかじめ回転させてから外積をとっても同じ結果になる。 $\bm{B}(\bm{a} \times \bm{b}) = (\bm{B}\bm{a}) \times (\bm{B}\bm{b})$
>- $\bm{B}$ はDCM（Direction Cosine Matrix）とも呼ばれる。回転行列の各列は、動座標系の基底ベクトルを静止座標系で表現したときの成分表示になっている。
>- $\bm{B}$ の時間微分 $\dot{\bm{B}}$ を左から動座標系のベクトルに掛けることは、回転の角速度ベクトル $\bm{\Omega}$ を外積で掛けることと等価である。すなわち、 $\dot{\bm{B}}\bm{A} = \bm{B}(\bm{\Omega} \times \bm{A})$ となる（ポアソンの定理、後述）。

もっとも覚えておくべきは、以下の変換です。静止座標系と動座標系の間の基本的な変換式になります。ベクトル量であれば、この変換は常に成り立ちます（オイラー角やその時間微分には成り立たない）。

>[!IMPORTANT]
>| 静止座標系 $\{\bm{e}_1, \bm{e}_2, \bm{e}_3\}$ での座標 | 変換 | 動座標系 $\{\bm{E}_1, \bm{E}_2, \bm{E}_3\}$ での座標 |
>| :---: | :---: | :---: |
>| <br>速度 $\bm{v}$ <br>角速度 $\bm{\omega}$ <br>その他ベクトル<br>（力、トルク、運動量、角運動量） |  $\bm{B}^T$ を掛ける<br>$\rightarrow$<br>$\leftarrow$<br>$\bm{B}$ を掛ける   | <br>速度 $\bm{V}$<br> 角速度 $\bm{\Omega}$<br>その他ベクトル<br>（力、トルク、運動量、角運動量） |

### ■ 回転座標系における速度と加速度の展開（式 10, 11, 12, 13）

座標系が回転している環境下での、位置ベクトルの微分（速度・加速度）の展開式です。本論文では、ベクトル積（外積）をブラケット記法 $[\bm{a}, \bm{b}] = \bm{a} \times \bm{b}$ で表現しています。

* **速度ベクトル（式 10）：**
  機体が角速度 $\bm{\Omega}$ (動座標系) または $\bm{\omega}$ (静止座標系) で回転しているとき、動径ベクトル $\bm{Q}$ の微分は次のようになります。
  $$\dot{\bm{q}} = \dot{\bm{r}} + \frac{d}{dt}(\bm{B}\bm{Q}) = \dot{\bm{r}} + \dot{\bm{B}}\bm{Q} + \bm{B}\dot{\bm{Q}} = \dot{\bm{r}} + \bm{B} [\bm{\Omega}, \bm{Q}] + \bm{B}\dot{\bm{Q}} \quad \text{--- (式 5)再掲}$$

機体内固定の点であれば $\dot{\bm{Q}} = \bm{0}$ となり、論文中の
$$
\dot{\bm{q}} = \dot{\bm{r}} + [\bm{\omega}, \bm{B}\bm{Q}] = \dot{\bm{r}} + \bm{B}[\bm{\Omega}, \bm{Q}] = \dot{\bm{r}} + [\bm{B}\bm{\Omega}, \bm{B}\bm{Q}] \quad \text{--- (式 10) $\dot{Q} = 0$ の場合}
$$
 と等価になります。また、 $\bm{B} [\bm{\Omega}, \bm{Q}] = [\bm{B}\bm{\Omega}, \bm{B}\bm{Q}]$ となる理由は、回転行列 $\bm{B}$ は外積の構造を保つためです。すなわち、2つのベクトルの外積をとってから回転しても、2つのベクトルをあらかじめ回転させてからその外積をとっても同じ結果になります。この(式10)も今後登場しませんのでなるほど、と読んでおいて大丈夫。(式5)はさらにこのあと加速度の式(式13)に展開されますので、そちらで詳しく解説します。

### ■ 角速度ベクトルとオイラー角の微分（式 11, 12）
$$\bm{\Omega} = \bm{B}^T \bm{\omega} \quad \text{--- (式 11)}$$
$$\bm{\omega} = \dot{\psi}\bm{e}_3 + \dot{\theta}\bm{E}_2^{(-2)} + \dot{\phi}\bm{E}_1^{(-1)} \quad \text{--- (式 12)}$$
* **意味:** 機体固定の瞬間角速度 $\bm{\Omega}$ と角速度 $\bm{\omega}$ の関係式、およびオイラー角の微分（$\dot{\psi}, \dot{\theta}, \dot{\phi}$）が角速度 $\bm{\omega}$ とどう結びついているかを示します。

(式12)は、この後出てこないので、各オイラー角の時間微分が変換途中の座標軸に対してどのように速度で構成されることに注意するだけでよいです。
(式11)は上の「IMPORTANT」で紹介しましたし、この後何度も出てきます。逆方向の変換と合わせて覚えておきます。動系から静止系へのベクトルの変換は、 $\bm{B}$ を掛けることで行われることを思い出しまししょう。各速度は「ベクトル」ですので、この規則が当てはまります（オイラー角には当てはまりません、オイラー角は「ベクトル」でなく、成分ごとの和やスカラー倍したベクトルは物理的に意味をなしません）。

$$\bm{\Omega} = \bm{B}^T \bm{\omega} \quad \text{--- (式 11)}\\
\bm{\omega} = \bm{B} \bm{\Omega} \quad \text{--- (式 11)'}$$

を構成するかを示す式です。これを式11の関係式と組み合わせることで、オイラー角の微分から角速度への変換行列 $\bm{\omega}_{\dot{\bm{x}}}(\psi, \theta)$ を導出することができます。
  
* **加速度ベクトル（式 13）：**
速度ベクトル（式10）をさらにもう一度時間微分することで、回転系における加速度方程式を得ます。

$$\ddot{\bm{q}} = \ddot{\bm{r}} + \bm{B} [\bm{\Omega}, [\bm{\Omega}, \bm{Q}]] + \bm{B} [\dot{\bm{\Omega}}, \bm{Q}] + 2\bm{B} [\bm{\Omega}, \dot{\bm{Q}}] + \bm{B}\ddot{\bm{Q}} \quad \text{--- (動座標系角速度ベクトル)} \bm{\Omega} \\
\ddot{\bm{q}} = \ddot{\bm{r}} + [\bm{\omega}, [\bm{\omega}, \bm{B}\bm{Q}]] + [\dot{\bm{\omega}}, \bm{B}\bm{Q}] + 2[\bm{\omega}, \bm{B}\dot{\bm{Q}}] + \bm{B}\ddot{\bm{Q}} \quad \text{--- (静止座標系角速度ベクトル)}\bm{\omega}$$

  これで、回転座標系における加速度の展開式が得られます（この計算詳細は後述します）。

#### 💡 各項の物理的な意味
1. **$\ddot{\bm{r}}$ : 重心の並進加速度**
   * 静止座標系に対するドローン重心全体の加速度です。
2. **$\bm{B} [\bm{\Omega}, [\bm{\Omega}, \bm{Q}]]$ または $[\bm{\omega}, [\bm{\omega}, \bm{B}\bm{Q}]]$ : 遠心力**
   * 回転運動によって、回転中心から外側へ向かって引き離すように作用する慣性加速度です。
3. **$\bm{B} [\dot{\bm{\Omega}}, \bm{Q}]$ または $[\dot{\bm{\omega}}, \bm{B}\bm{Q}]$ : 回転慣性力**
   * 回転の角速度が変化（角加速度 $\dot{\bm{\Omega}}$ または $\dot{\bm{\omega}}$ が発生）したときに、回転の接線方向に発生する加速度です。
4. **$2\bm{B} [\bm{\Omega}, \dot{\bm{Q}}]$ または $2[\bm{\omega}, \bm{B}\dot{\bm{Q}}]$ : Colioris力**
   * 回転している座標系の中で、対象の点が相対速度 $\dot{\bm{Q}}$ を持って移動する際に、回転軸と速度の双方に直交する方向に受ける見かけの加速度です。
5. **$\bm{B}\ddot{\bm{Q}}$ : 相対加速度**
   * ドローンの機体固定座標系から見た、動点の純粋な相対加速度です。

難しくなりましたが、この汎用式（式13）は、この論文の最後まで（剛体のみを扱うため）活用されません。一度眺めて貰えばよいでしょう。
すなわち、本論文では $\dot{\bm{Q}} = \bm{0}, \ddot{\bm{Q}} = \bm{0}$ ため、4.のColioris力の項、5.の相対加速度の項は消えます。さらに、$\bm{r}$ をドローンの重心位置としているため（ $\Sigma m_i \bm{Q}_i = 0$ ）、 $\bm{Q}$ がそのまま関与する 2.の遠心力、3.のオイラー加速度の項も釣り合って消えます。すると、実際に本論文で使われる重心の並進に関する式は、

$$\ddot{\bm{q}} = \ddot{\bm{r}} \quad \text{--- (式 13)’}$$

となるだけです。動座標系 $\dot{\bm{Q}}$ に起因する項は消えます。

ただし、ここから、動座標系での並進の運動方程式を立式する際には、やはり見かけの力が（再度）発生します。
重心の速度 $\bm{v} = \dot{\bm{r}}$ とし、

$$
\bm{f} = m\dot{\bm{v}} \quad \text{--- (*1)ニュートンの方程式(静止座標系)}
$$

という重心に対する静止座標でのニュートンの方程式を立式します（ $\bm{f} \in w$ ）。
重心の速度 $\bm{v} = \dot{\bm{r}} \in w$ を動座標系で表して $\bm{V} \in W$ とすると $\bm{v} = \bm{B}\bm{V}$ となるため、輸送定理（すぐ後に解説）により、（式13'）は、

$$\ddot{\bm{r}} = \dot{\bm{v}} = \bm{B} (\dot{\bm{V}} + \bm{\Omega} \times \bm{V})$$

となり、重心についても動座標系で立式した場合のベクトルには慣性力がかかります。(*)は、

$$
\bm{f} = m \dot{\bm{v}} = m \bm{B} (\dot{\bm{V}} + \bm{\Omega} \times \bm{V})
$$
$\bm{F} = \bm{B}^T \bm{f} \in W$ として両辺に $\bm{B}^T$ を掛けて動座標系で表すと、
$$
\bm{F} = m\dot{\bm{V}} + m \bm{\Omega} \times \bm{V}\quad \text{--- (*1')ニュートンの方程式(動座標系)}
$$

となり、コリオリ力に相当する項が再度現れます。原点（重心）の並進に対してかかるコリオリ力です。一旦、剛体条件で消えたのですが、並進の中にも含まれていたのですね。そして、この式が、ドローンの運動方程式の基礎となり、後の状態方程式の導出に繋がっていきます。


>[!NOTE] ポアソンの関係式と輸送定理
>回転行列 $\bm{B}(t)$、その動座標系での回転角速度ベクトルを $\bm{\Omega} \in W$ 、対応する静止座標系での回転各速度ベクトルを $\bm{\omega} = \bm{B}\bm{\Omega} \in w$ とすると、任意の動座標ベクトル  $\bm{Q} \in W$ 、その対応する静止座標ベクトル $\bm{q} = \bm{B}\bm{Q} \in w$ に対して、
>**ポアソンの関係式** ：
>$$
>\begin{align*}
>\dot{\bm{B}}\bm{Q} &= \bm{B} (\bm{\Omega} \times \bm{Q})\\
> &= \bm{\omega} \times (\bm{B}\bm{Q})\\
> &= \bm{\omega} \times \bm{q}
>\end{align*}
>$$
>
>**輸送定理** ：（ポアソンの関係式を使って容易に証明可）
>$$
>\dot{\bm{q}} = \bm{B} (\bm{\Omega} \times \bm{Q} + \dot{\bm{Q}})
>$$
>
>すなわち、「静止座標での時間微分を動座標の時間微分に変換する際、 $\bm{\Omega}$ の外積を掛けたものをプラスする」。
>
>**直感的理解** ：
>動座標系がZ軸周りに回転していた場合、動座標系の+x軸方向に動いている点は、静止座標系から見ると、回転の影響で+y軸方向にも動いているように見えることがわかります。これが $\bm{\Omega} \times \bm{Q}$ です。

これらを使って、

#### 💡 式 13 の数学的導出メモ

ここでは、(式5)の前半式から輸送定理を用いて(式13)導出してみます。
$$
\begin{align*}
\dot{\bm{q}} &= \dot{\bm{r}} + \frac{d}{dt}(\bm{B}\bm{Q}) \quad \text{--- (式 5)再掲} \\
&= \dot{\bm{r}} + \bm{B}(\Omega \times \bm{Q} + \dot{\bm{Q}}) \quad \because \text{輸送定理}
\end{align*}
$$
さらに時間微分、ポアソンの関係式を使って展開していきます。
$$
\begin{align*}
\ddot{\bm{q}} &= \ddot{\bm{r}} + \dot{\bm{B}}(\Omega \times \bm{Q} + \dot{\bm{Q}}) + \bm{B} \frac{d}{dt}(\Omega \times \bm{Q} + \dot{\bm{Q}}) \\
&= \ddot{\bm{r}} + \bm{B}(\Omega \times (\Omega \times \bm{Q})) + \bm{B}(\Omega \times \dot{\bm{Q}}) + \bm{B} \frac{d}{dt}(\Omega \times \bm{Q}) + \bm{B}\ddot{\bm{Q}} \\
&= \ddot{\bm{r}} + \bm{B}(\Omega \times (\Omega \times \bm{Q})) + 2\bm{B}(\Omega \times \dot{\bm{Q}}) + \bm{B}(\dot{\Omega} \times \bm{Q}) + \bm{B}\ddot{\bm{Q}}
\end{align*}
$$

再度、論文の式(13)を確認してみましょう。

$$
\begin{align*}
\ddot{\bm{q}} &= \ddot{\bm{r}} + \bm{B}\ddot{\bm{Q}} + 2\dot{\bm{B}}\dot{\bm{Q}} + \ddot{\bm{B}}\bm{Q} \quad \text{--- 式13}\\
&= \ddot{\bm{r}} + \bm{B} [\bm{\Omega}, [\bm{\Omega}, \bm{Q}]] + \bm{B} [\dot{\bm{\Omega}}, \bm{Q}] + 2\bm{B} [\bm{\Omega}, \dot{\bm{Q}}] + \bm{B}\ddot{\bm{Q}} \quad \text{--- (動座標系角速度ベクトル)} \bm{\Omega}\\
&= \ddot{\bm{r}} + [\bm{\omega}, [\bm{\omega}, \bm{B}\bm{Q}]] + [\dot{\bm{\omega}}, \bm{B}\bm{Q}] + 2[\bm{\omega}, \bm{B}\dot{\bm{Q}}] + \bm{B}\ddot{\bm{Q}} \quad \text{--- (静止座標系角速度ベクトル)}\bm{\omega}
\end{align*}
$$

外積の記号は違いますが、同じ式が得られました。 $B [\Omega, Q] = [B\Omega, BQ] （外積の保存）= [\omega, BQ]$ から二つの式は同じにになります。これで、(式13)の導出が確認できました（ただし、この式は今後登場することはありません。見かけの力とその原因を洗い出した、という意味があります）。


---

## 3.2節：剛体の回転による方程式（オイラーの方程式）（式 14〜18）

第3.2節では、ドローンの姿勢変化と力の伝達、およびそれらを統一した非線形の状態方程式（運動方程式）を組み立てます。

### ■ オイラーの運動方程式（式 14, 15）
$$
\begin{align*}
\bm{h} &= \bm{I}\bm{\omega} \quad \text{--- (式 14)、角運動量はモーメントと角速度の積} \\
\dot{\bm{h}} &= \bm{\tau} \quad \text{--- (式 15)、トルクが角運動量の時間変化を生む(*2) オイラーの方程式（静止座標）}
\end{align*}
$$
* **意味:** 静止座標系における角運動量 $\bm{h}$ とトルク $\bm{\tau}$ の関係です。

静止座標系では、慣性モーメント $\bm{I} = \bm{I}(\bm{x})$ はオイラー角 $\bm{x} = (\psi, \theta, \phi)^T$ に依存する扱いにくい量になります。よって、重心を原点とする動座標系で扱うことで、 $\hat{\bm{I}}$ という姿勢に依存しない主軸対角行列に変換できます。$\hat{\bm{I}}$ はドローンの質量分布に依存する定数行列であり、ドローンの形状や質量配置が変わらない限り一定です。ちなみに、$\bm{I}$ と $\hat{\bm{I}}$ は、回転行列 $\bm{B}$ を用いて $\bm{I} = \bm{B}\hat{\bm{I}}\bm{B}^T$ という関係で結びついています。

(式15)の両辺を動座標系で記述します。 $\bm{H}, \bm{\Omega}, \bm{T} \in W$ を動座標系での角運動量、角速度、トルクとすると、これらはベクトルの変換規則に従って、
$$
\begin{align*}
\bm{h} &= \bm{B}\bm{H}\\
\bm{\tau} &= \bm{B} \bm{T}\\
\bm{\omega} &= \bm{B}\bm{\Omega}
\end{align*}
$$
 であり、(式14)にそれぞれ代入して時間微分を計算すると、輸送定理より
が成り立ち、これが(式15)よりトルクになるため、
$$\bm{B}(\bm{\Omega} \times \bm{H} + \dot{\bm{H}}) = \bm{B} \bm{T}$$

が得られます。両辺に $\bm{B}^T$ を掛けて、

$$\bm{\Omega} \times \bm{H} + \dot{\bm{H}} = \bm{T}\quad \text{--- (式 15)の動座標系版}$$

さらに、(式16) $\bm{H} = \hat{\bm{I}}\bm{\Omega}, \dot{\bm{H}} = \hat{\bm{I}}\dot{\bm{\Omega}}$ として、オイラーの運動方程式が得られます。これら２つの方程式で、剛体ドローンの姿勢変化と速度に対する力とトルクの関係は、動座標系で完全に表現できます。繰り返しですが、動座標系にするのは、慣性モーメントを姿勢によらない定数として対角化したいためです。

>[!IMPORTANT]
>**剛体の運動方程式（動座標系）**
>$$
>\begin{align*}
>\bm{F} &= m\dot{\bm{V}} + m \bm{\Omega} \times \bm{V}\quad \text{--- (*1')ニュートンの方程式}\\
>\bm{T} &= \hat{\bm{I}}\dot{\bm{\Omega}} + \bm{\Omega} \times (\hat{\bm{I}}\bm{\Omega}) \quad \text{---(*2') オイラーの方程式（(式 14,15)の動座標系版）}
>\end{align*}
>$$

### ■ ドローンのオイラー角空間（式 19〜21）
$span\{\bm{\epsilon}_1, \bm{\epsilon}_2, \bm{\epsilon}_3\}$ をドローンのオイラー角空間と定義し、オイラー角 $\psi, \theta, \phi$ をこの空間の座標として定義します。これにより、ドローンの姿勢を表す状態変数 $\bm{x}$ を以下のように定義できます。
$$
\begin{align*}
\bm{x} &= \psi \bm{\epsilon}_1 + \theta \bm{\epsilon}_2 + \phi \bm{\epsilon}_3 = \psi \bm{\epsilon}_1 + \bm{\eta} \quad \text{--- (式 19)}\\
\dot{\bm{x}} &= \dot{\psi} \bm{\epsilon}_1 + \dot{\theta} \bm{\epsilon}_2 + \dot{\phi} \bm{\epsilon}_3 = \dot{\psi} \bm{\epsilon}_1 + \dot{\bm{\eta}} \quad \text{--- (式 20)}
\end{align*}
$$
ここで、 $\psi$ はドローンのヨー角で、この角だけ分離した扱いになっています。これは後に、モーター故障の際にこの角だけ制御を切り離して他の角を制御する、という戦略を見据えた定式化です。 $\bm{\eta} = \theta \bm{\epsilon}_2 + \phi \bm{\epsilon}_3$ は、ピッチとロールの角度をまとめたベクトルです。

さらに、この3つの状態変数とその微分、すなわち $(\bm{x}, \dot{\bm{x}}) \in \mathbb{R}^6$ という6変数を用いて状態空間とします。




### ■ ラグランジアン $L$ と一般化力（式 22, 23）

ここで、ラグランジアンが唐突に登場しますが、やろうとしていることは先の姿勢（オイラー角）に関する状態空間に対応する、

- 一般化座標（ $\bm{x} = (\psi, \theta, \phi)$ ）
- 一般化速度（ $\dot{\bm{x}} = (\dot{\psi}, \dot{\theta}, \dot{\phi}) $ ）
- 対応する一般化力（ $\bm{f} = \bm{B}(\bm{x})\bm{F}_{\text{rot}}(\bm{u})$ ）

を定義して、ニュートン・オイラー方程式を一般化座標系で表現することです。そして $\bm{F}_{\text{rot}}(\bm{u})$ は、ドローンのモーターから発生する回転トルクを表すベクトルで、制御入力 $\bm{u}$ に依存します。これらから、
>[!IMPORTANT]
>入力 $\bm{u}$ （モーターへの制御信号）に対応する出力 $\ddot{\bm{x}}$ （状態空間内の状態ベクトルの遷移）の「感度」を導出することが目的

です。これによって、入力 $\bm{u}$ に対するドローンの姿勢をオイラー角空間で制御できるようになります。

では、ラグランジアンの式を解説しますが、ここの理解を飛ばしても問題ありません。通常のニュートン・オイラー方程式からも、同様の結果が導かれます。

$$L = \frac{1}{2} m \langle \dot{\bm{r}}, \dot{\bm{r}} \rangle + \frac{1}{2} \langle \hat{\bm{I}}\bm{\Omega}, \bm{\Omega} \rangle - mg \langle \bm{r}, \bm{e}_3 \rangle \quad \text{--- (式 22)}\\
\langle \bm{B}(\bm{x})\bm{F}_{\text{rot}}(\bm{u}), \delta\bm{\omega} \rangle = \langle \bm{\omega}_{\dot{\bm{x}}}(\psi, \theta)^T \bm{B}(\bm{x})\bm{F}_{\text{rot}}(\bm{u}), \delta\dot{\bm{x}} \rangle \quad \text{--- (式 23)}$$
* **第22式:** 並進の運動エネルギー、回転の運動エネルギー、重力ポテンシャルから構成されるラグランジアンです。
* **第23式:** 仮想仕事の原理（の時間微分）に基づき、モータによるモーメント $\bm{F}_{\text{rot}}$ をオイラー角 $\bm{x}$ の一般化力へ変換します。

* **変換行列 $\bm{\omega}_{\dot{\bm{x}}} = \bm{\omega}_{\dot{\bm{x}}}(\psi, \theta)$（式 28）：**
$$
\bm{\omega}_{\dot{\bm{x}}} = \bm{\omega}_{\dot{\bm{x}}}(\psi, \theta) = \begin{bmatrix} 0 & -\sin\psi & \cos\theta \cos\psi \\ 0 & \cos\psi & \cos\theta \sin\psi \\ 1 & 0 & -\sin\theta \end{bmatrix} \quad \text{--- (式 28)}
$$

この(式28)がポイントになります。この式は単なる計算のようでありますが深い意味を持ちます。



- **オイラー角の微分 $\dot{\bm{x}}$ と角速度 $\bm{\omega}$ の関係を表す変換行列** です。実は、角速度 $\bm{\omega}$ はオイラー角の微分 $\dot{\bm{x}}$ の **線形変換** として表すことができます。その変換の表現行列が $\bm{\omega}_{\dot{\bm{x}}}$ です。$\bm{\omega} = \bm{\omega}_{\dot{\bm{x}}}(\psi, \theta) \dot{\bm{x}}$ 、すなわち（3次元ベクトル）＝（3x3行列） $\times$ （3成分列）という関係が成り立ちます。
- そして、その **変換行列** $\bm{\omega}_{\dot{\bm{x}}}$ は変数 **$(\psi, \theta)$ のみ** で決まります。
- さらに、（明示的に論文に記述がないですが）これを動座標系で計算した、$\bm{\Omega} = \bm{B}^T \bm{\omega} = \bm{B}^T \bm{\omega}_{\dot{\bm{x}}}(\psi, \theta) \dot{\bm{x}}$ を計算すると、今度は、

$$
\begin{align*}
\bm{\Omega}_{\dot{\bm{x}}} &= \bm{\Omega}_{\dot{\bm{x}}}(\theta, \phi) \quad (= \bm{B}^T \bm{\omega}_{\dot{\bm{x}}}(\psi, \theta)) \quad \text{--- (式 28)の動座標系版}\\
&= \begin{bmatrix}
-\sin\theta & 0 & 1 \\
\sin\phi \cos\theta & \cos\phi & 0 \\
\cos\phi \cos\theta & -\sin\phi & 0
\end{bmatrix} \quad \text{--- (式 28)の動座標系版}
\end{align*}
$$

となり（箱庭physics参照）、  **$\psi$ が式から消えてしいまいます！** したがって、角速度ベクトルの関係は次のように書き下せます。

$$
\begin{align*}
\bm{\Omega} &= \bm{\Omega}_{\dot{\bm{x}}}(\theta, \phi) \dot{\bm{x}}\\
\begin{bmatrix}
\Omega_1 \\
\Omega_2 \\
\Omega_3
\end{bmatrix}
&=\begin{bmatrix}
-\sin\theta & 0 & 1 \\
\sin\phi \cos\theta & \cos\phi & 0 \\
\cos\phi \cos\theta & -\sin\phi & 0
\end{bmatrix}
\begin{bmatrix}
\dot{\psi} \\
\dot{\theta} \\
\dot{\phi}
\end{bmatrix}, \quad \text{（箱庭physicsでは動座標角速度を$\Omega = [p,q,r]$と表現）}
\end{align*}
$$

（検算として、実際に $\bm{\Omega} = \bm{B}^T \bm{\omega}_{\dot{\bm{x}}}(\psi, \theta)$ を行列計算してみると、 

$$
\begin{align*}
\bm{B}^T \bm{\omega}_{\dot{\bm{x}}}
&=
\begin{bmatrix}
\cos\psi \cos\theta & \sin\psi \cos\theta & -\sin\theta \\
\cos\psi \sin \theta \sin \phi - \sin\psi \cos\phi & \sin\psi \sin\theta \sin\phi + \cos\psi \cos \phi & \cos\theta \sin\phi \\
\cos\psi \sin\theta \cos\phi + \sin\psi \sin\phi & \sin\psi \sin\theta \cos\phi - \cos\psi \sin\phi & \cos\theta \cos\phi
\end{bmatrix}
\begin{bmatrix}
0 & -\sin\psi & \cos\theta \cos\psi \\
0 & \cos\psi & \cos\theta \sin\psi \\
1 & 0 & -\sin\theta
\end{bmatrix} \\
&=
\begin{bmatrix}
-\sin\theta & 0 & 1 \\
\sin\phi\cos\theta & \cos\phi & 0 \\
\cos\phi\cos\theta & -\sin\phi & 0
\end{bmatrix}
\; (= \bm{\Omega}_{\dot{\bm{x}}}(\theta,\phi))
\end{align*}
$$
検算終わり）

この計算から、ヨー角（$\psi$）を式から追払うことができ、動座標系での角速度 $\bm{\Omega}$ は、オイラー角の時間変化に対して線形で、ピッチとロールの角度（$\theta, \phi$）のみに依存していることになります。これによって、ヨー角を制御から独立させることができるのです。この逆行列を用いることで、動座標系の角速度 $\bm{\Omega}$ からオイラー角の時間微分 $\dot{\bm{x}}$ に変換することもできます。すなわち、 $\bm{\omega}_{\dot{\bm{x}}}, \bm{\Omega}_{\dot{\bm{x}}}$ の逆行列をそれぞれ計算して、以下のようになります。

$$
\begin{align*}
\dot{\bm{x}} &= \bm{\omega}_{\dot{\bm{x}}}(\psi, \theta)^{-1} \bm{\omega}\\
\begin{bmatrix}
\dot{\psi} \\
\dot{\theta} \\
\dot{\phi}
\end{bmatrix} &= \begin{bmatrix}
\cos\psi\tan\theta & \sin\psi\tan\theta & 1 \\
-\sin\psi & \cos\psi & 0 \\
\cos\psi\sec\theta & \sin\psi\sec\theta & 0
\end{bmatrix}
\begin{bmatrix}\omega_1 \\
\omega_2 \\
\omega_3\end{bmatrix}
\end{align*}
$$

$$
\begin{align*}
\dot{\bm{x}} &= \bm{\Omega}_{\dot{\bm{x}}}(\theta, \phi)^{-1} \bm{\Omega}\\
\begin{bmatrix}
\dot{\psi} \\
\dot{\theta} \\
\dot{\phi}
\end{bmatrix} &= \begin{bmatrix}
0 & \sin\phi\sec\theta & \cos\phi\sec\theta \\
0 & \cos\phi & -\sin\phi \\
1 & \sin\phi\tan\theta & \cos\phi\tan\theta
\end{bmatrix} 
\begin{bmatrix}\Omega_1 \\
\Omega_2 \\
\Omega_3\end{bmatrix}
\end{align*}
$$

これにより、求めるオイラー角の変化率を角速度から得ることができます。さらにそれを微分することで、オイラー角の加速度 $\ddot{\bm{x}}$ も角加速度 $\dot{\bm{\omega}}$ から求められます。これが、感度（以下の章で登場する $\bm{Z}(\bm{\eta})$ ）の肝になります。


### ■ 回転の明示的な状態方程式（定理 1 / 式 29〜32）
$$\frac{d}{dt} \begin{bmatrix} \bm{x} \\ \dot{\bm{x}} \end{bmatrix} = \begin{bmatrix} \dot{\bm{x}} \\ \bm{Y}(\bm{\eta}, \dot{\bm{x}}) + \bm{Z}(\bm{\eta})\bm{F}_{\text{rot}}(\bm{u}) \end{bmatrix} \quad \text{--- (式 30)}$$

* **制御入力ゲイン行列 $\bm{Z}(\bm{\eta})$（式 31）:**
  $$\bm{Z}(\bm{\eta}) = (\bm{B}\hat{\bm{I}}\bm{B}^T \cdot \bm{\omega}_{\dot{\bm{x}}})^{-1} \bm{B} \quad \text{--- (式 31)}$$
 * **慣性項 $\bm{Y}(\bm{\eta}, \dot{\bm{x}})$（式 32）:**
  $$\bm{Y}(\bm{\eta}, \dot{\bm{x}}) = -(\bm{B}\hat{\bm{I}}\bm{B}^T \bm{\omega}_{\dot{\bm{x}}})^{-1} (\dot{\bm{B}}\hat{\bm{I}}\bm{B}^T \dot{\bm{\omega}}_{\dot{\bm{x}}}  + \bm{B}\hat{\bm{I}}\bm{B}^T \bm{\omega}_{\dot{\bm{x}}})\dot{\bm{x}} \quad \text{--- (式 32)}$$ 

  前章の解説から、この $\bm{Z}(\bm{\eta})$ は、モーターから発生するトルク $\bm{F}_{\text{rot}}$ が、オイラー角の加速度 $\ddot{\bm{x}}$ にどのように影響を与えるかを表すゲイン行列です。 $\bm{Z}$ は、ドローンの姿勢（特にピッチとロールの角度）に依存して変化しますが、ヨー角には依存しません。これが、ヨー角を制御から切り離すことができる理由です。論文では静止座標から求めていますが、動座標系で計算しても同様の結果が得られます（より簡便だと思う）。

>[!NOTE]
> **制御入力ゲイン行列 $\bm{Z}(\bm{\eta})$（式 31）の求め方:**
>
> ここでは、オイラー方程式を出発点にして計算してみます（論文とは違う道筋で計算。論文の計算は後に詳説）。
>$$
>\bm{T} = \hat{\bm{I}}\dot{\bm{\Omega}} + \bm{\Omega} \times (\hat{\bm{I}}\bm{\Omega}) \quad \text{---(*2') オイラーの方程式（(式 14,15)）}
>$$
>ここで、$T = F_{rot}$ であり、$M = \bm{\Omega}_{\dot{\bm{x}}}$ とし、$\bm{\Omega} = M \dot{\bm{x}}$ と記述すると、
>$$
>\begin{align*}
>\bm{T} = \hat{\bm{I}}(M \ddot{\bm{x}} + \dot{M} \dot{\bm{x}}) + (M\dot{\bm{x}}) \times (\hat{\bm{I}} M \dot{\bm{x}})
>\end{align*}
>$$
>この式を $\ddot{\bm{x}}$ について解く。
>$$
>\begin{align*}
>\hat{\bm{I}}M \ddot{\bm{{x}}} &= \bm{T} - \hat{\bm{I}} \dot{M} \dot{\bm{x}} - (M\dot{\bm{x}}) \times (\hat{\bm{I}} M \dot{\bm{x}})\\
>\ddot{\bm{x}} &= (M^{-1} \hat{\bm{I}}^{-1}) \bm{T} - (M^{-1}\hat{\bm{I}}^{-1})(\hat{\bm{I}}\dot{M} \dot{\bm{x}} + (M\dot{\bm{x}}) \times (\hat{\bm{I}} M \dot{\bm{x}}))\\
>&= \bm{Z}(\bm{\eta}) \bm{T} + \bm{Y}(\bm{\eta}, \dot{\bm{x}})
>\end{align*}
>$$
>
>ここで、 
>$$
>\begin{align*}
>\bm{Z}(\bm{\eta}) &= M^{-1} \hat{\bm{I}}^{-1} \quad \text{--- (式 *3)}\\
>\bm{Y}(\bm{\eta}, \dot{\bm{x}}) &= - (M^{-1}\hat{\bm{I}}^{-1})(\hat{\bm{I}}\dot{M} \dot{\bm{x}} + (M\dot{\bm{x}}) \times (\hat{\bm{I}} M \dot{\bm{x}})) \quad \text{--- (式 *4)}
>\end{align*}
>$$
>
>と定義されます。これが、オイラーの方程式から求めることができる、ドローンの回転に関する状態方程式の形式になります。諸元は以下を使えば実際に計算できます。
>$$
>\dot{M} = \begin{bmatrix}
>0 & -\dot{\phi}\cos\phi  & -\dot{\phi}\sin\phi \cos\theta - \dot{\theta}\cos\phi \sin\theta \\
>0 & -\dot{\phi}\sin\phi  & \dot{\phi}\cos\phi \cos\theta  - \dot{\theta}\sin\theta \sin\phi  \\
>0 & 0 & -\dot{\theta} \cos\theta
>\end{bmatrix}, \quad
>M^{-1} = \begin{bmatrix}
>0 & \sin\phi\sec\theta & \cos\phi\sec\theta \\
>0 & \cos\phi & -\sin\phi \\
>1 & \sin\phi\tan\theta & \cos\phi\tan\theta
>\end{bmatrix}, \quad 
>\hat{\bm{I}}^{-1} = \begin{bmatrix}
>1/I_{11} & 0 & 0 \\
>0 & 1/I_{22} & 0 \\
>0 & 0 & 1/I_{33}
>\end{bmatrix} 
>$$
>
>また、式 *4 の外積項 $(M\dot{\bm{x}}) \times (\hat{\bm{I}}M\dot{\bm{x}})$ を、スキュー対称行列（外積と同じ働きをする交代行列）で表現することで、外積記号を消して完全に行列形式に統一できます。任意のベクトル $\bm{a}$ に対し、スキュー対称行列 $[\bm{a}]_\times$ を定義する：
>$$
>\bm{a} \times \bm{b} = [\bm{a}]_\times \bm{b}, \quad \text{ここで } \quad [\bm{a}]_\times = \begin{bmatrix} 0 & -a_3 & a_2 \\ a_3 & 0 & -a_1 \\ -a_2 & a_1 & 0 \end{bmatrix}
>$$
>
>したがって、式 *4 の外積項は、こう書ける。
>$$(M\dot{\bm{x}}) \times (\hat{\bm{I}}M\dot{\bm{x}}) = [M\dot{\bm{x}}]_\times \hat{\bm{I}}M\dot{\bm{x}} = [M\dot{\bm{x}}]_\times \hat{\bm{I}}M \dot{\bm{x}}$$
>
>**スキュー行列版**
>$$
>\bm{Y}(\bm{\eta}, \dot{\bm{x}}) = -(M^{-1}\hat{\bm{I}}^{-1})(\hat{\bm{I}}\dot{M} + [M\dot{\bm{x}}]_\times \hat{\bm{I}}M) \dot{\bm{x}} \quad \text{--- (式 *4')}
>$$


### ■ 式 *4 から式 32を導く

ここでは、式 *4 から論文の式 32 を導出する詳細なプロセスを示します。

#### ステップ2：スキュー行列による外積の統一表現

スタート **スキュー行列版**（式 *4'）

$$\boxed{\bm{Y} = -(M^{-1}\hat{\bm{I}}^{-1})(\hat{\bm{I}}\dot{M} + [M\dot{\bm{x}}]_\times \hat{\bm{I}}M)\dot{\bm{x}}}$$

#### ステップ4：座標変換による式 32 の導出

この式 *4 のスキュー行列版に対して、座標変換 $M = \bm{B}^T \bm{\omega}_{\dot{\bm{x}}}$ を適用します。

**変換関係：**
$$\begin{align*}
M &= \bm{B}^T \bm{\omega}_{\dot{\bm{x}}} \\
\dot{M} &= \dot{\bm{B}}^T \bm{\omega}_{\dot{\bm{x}}} + \bm{B}^T \dot{\bm{\omega}}_{\dot{\bm{x}}}
\end{align*}$$

**逆行列の性質：**
$$M^{-1} = \bm{\omega}_{\dot{\bm{x}}}^{-1} \bm{B}$$

**第1項の変換：**
$$-(M^{-1}\hat{\bm{I}}^{-1})(\hat{\bm{I}}\dot{M})\dot{\bm{x}} = -(\bm{\omega}_{\dot{\bm{x}}}^{-1}\bm{B}\hat{\bm{I}}^{-1}\hat{\bm{I}}\dot{\bm{B}}^T\bm{\omega}_{\dot{\bm{x}}})\dot{\bm{x}}$$

左から $\bm{\omega}_{\dot{\bm{x}}}$ を掛け、右から $\bm{\omega}_{\dot{\bm{x}}}^{-1}$ を掛けることで、静止座標系での形：

$$\dot{\bm{B}}\hat{\bm{I}}\bm{B}^T\dot{\bm{\omega}}_{\dot{\bm{x}}}$$

**第2項の変換：**
スキュー行列の座標変換性質：

$$\bm{B}[M\dot{\bm{x}}]_\times \hat{\bm{I}}M = [B M\dot{\bm{x}}]_\times \bm{B}\hat{\bm{I}}M = [\bm{\omega}]_\times \bm{B}\hat{\bm{I}}\bm{B}^T\bm{\omega}$$

ここで $\bm{\omega} = \bm{\omega}_{\dot{\bm{x}}}\dot{\bm{x}}$ です。

#### ステップ5：式 32 の確定

上記の変換をまとめると、静止座標系での形：
$$\boxed{\bm{Y}(\bm{\eta}, \dot{\bm{x}}) = -(\bm{B}\hat{\bm{I}}\bm{B}^T\bm{\omega}_{\dot{\bm{x}}})^{-1}(\dot{\bm{B}}\hat{\bm{I}}\bm{B}^T\dot{\bm{\omega}}_{\dot{\bm{x}}} + \bm{B}\hat{\bm{I}}\bm{B}^T\bm{\omega}_{\dot{\bm{x}}})\dot{\bm{x}}} \quad \text{--- (式 32)}$$

### ■ 並進の明示的な状態方程式（定理 2 / 式 33）
$$\frac{d}{dt} \begin{bmatrix} \bm{r} \\ \dot{\bm{r}} \end{bmatrix} = \begin{bmatrix} \dot{\bm{r}} \\ -g \bm{e}_3 + \frac{1}{m} \bm{B}(\bm{\phi}_1(t, (\bm{x}_0, \dot{\bm{x}}_0)^T, \bm{u})) \bm{F}_{\text{tra}}(\bm{u}) \end{bmatrix}$$
* **意味:** 重力加速度 $-g\bm{e}_3$ と、回転行列 $\bm{B}$ によって静止座標系に投影されたモータの合計推力 $\bm{F}_{\text{tra}}$ に基づく、重心位置の並進運動方程式です。

### ■ モータ配置と伝達行列（式 34〜47）
モータの回転数 $\omega_{Mi}$ （入力ゲイン $\bm{u}_4 = (\omega_{M1}^2, \omega_{M2}^2, \omega_{M3}^2, \omega_{M4}^2)^T$）からモーメントや推力を生成する変換行列を定義します。

* **クアッドコプタのモーメント伝達行列 $\bm{S}_{\text{rot}4}$（式 36）：**
  $$\bm{S}_{\text{rot}4} = \begin{bmatrix} 0 & -l k_F & 0 & l k_F \\ l k_F & 0 & -l k_F & 0 \\ -k_M & k_M & -k_M & k_M \end{bmatrix}$$
  * 1行目: ロールモーメント（左右の推力差 $\times$ アーム長 $l$）
  * 2行目: ピッチモーメント（前後の推力差 $\times$ アーム長 $l$）
  * 3行目: ヨーモーメント（時計回りと反時計回りモータの反トルク $k_M$ の差）
* **並進推力伝達行列 $\bm{S}_{\text{tra}4}$（式 45）：**
  $$\bm{S}_{\text{tra}4} = \begin{bmatrix} 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \\ k_F & k_F & k_F & k_F \end{bmatrix}$$

## 3. 4章：動作点と平衡点の定義（式 50〜55）

4章では、安定飛行または故障時における目標の飛行状態（動作点および平衡点）を定義します。
  $$\dot{\bm{x}}_{op} = \bm{0}_3$$
  $$\bm{Z}(\bm{\eta}_{op})\bm{S}_{\text{rot}2p}\bm{u}_{2p(op)} = \bm{0}_3$$
* **速度・高度（並進）に関する条件（式 51）:**
  $$\dot{\bm{r}}_{op} = \text{constant}$$
  $$\langle \bm{e}_3, -g\bm{e}_3 + \frac{1}{m}\bm{B}(\bm{x}_{op})\bm{S}_{\text{tra}2p}\bm{u}_{2p(op)} \rangle = c$$
  * $c = 0$ のときは **「定高度飛行 (Constant altitude flight)」** の状態となります。

### ■ 平衡点 (Equilibrium Point / Hovering)
定高度飛行状態（$c = 0$）において、さらに水平方向の加速度もゼロ（$\ddot{r}_1 = \ddot{r}_2 = 0$）であるならば、この動作点を**「平衡点」**と呼び、**ホバリング状態**に対応します。
* **水平加速度の式（式 53, 54）：**
  $$\ddot{r}_1 = \left\langle \bm{e}_1, \frac{1}{m}\bm{B}(\bm{x}_{op})\bm{S}_{\text{tra}2p}\bm{u}_{2p(op)} \right\rangle = 0$$
  $$\ddot{r}_2 = \left\langle \bm{e}_2, \frac{1}{m}\bm{B}(\bm{x}_{op})\bm{S}_{\text{tra}2p}\bm{u}_{2p(op)} \right\rangle = 0$$

---

## 4. 4章：モータ故障時における墜落回避制御（式 56〜57, 85〜91）

一部のモータが完全停止（$\omega_{Mi} = 0$）した状態でも、墜落を防ぐための制御可能条件を定式化します。

### ■ 正常時のフル状態（4×4）システム（定理 3 / 式 56〜57）
目標となる角加速度・鉛直加速度 $\bm{b}_4 = (\ddot{\psi}, \ddot{\theta}, \ddot{\phi}, \ddot{r}_3 + g)^T$ と、モータ入力 $\bm{u}_4$ の関係は次のように表されます。
$$\bm{A}_4(\bm{\eta})\bm{u}_4 = \bm{b}_4$$
ここで、行列 $\bm{A}_4(\bm{\eta})$ は以下のように構成されます。
$$\bm{A}_4(\bm{\eta}) = \begin{bmatrix} \bm{Z}(\bm{\eta})\bm{S}_{\text{rot}4} \\ \bm{e}_3^T \frac{1}{m}\bm{B}(\bm{x})\bm{S}_{\text{tra}4} \end{bmatrix}$$
* $\bm{A}_4(\bm{\eta})$ が正則（逆行列が存在する）であれば、一意のモータ速度 $\bm{u}_4 = \bm{A}_4^{-1}\bm{b}_4$ を求めてドローンを意のままに制御できます。

### ■ 1モータ故障時の縮小システム（式 85〜91）
クアッドコプタにおいてモータが1つ故障した場合、入力ベクトルは3次元になります。4次元の出力（ヨー、ピッチ、ロール、高度）をすべて制御することは不可能となるため、**ヨー角の制御（$\ddot{\psi}$）を放棄する（Type II 回避状態）**ことで、残りの3要素を制御します。

ヨーの行を削除した縮小システム（式 91）は次の通りです。
$$\bm{A}_{3}^1(\bm{\eta})\bm{u}_{3}^1 = \bm{b}_3$$
$$\begin{bmatrix} \bm{e}_2^T \bm{Z}(\bm{\eta})\bm{S}_{\text{rot}4}^1 \\ \bm{e}_3^T \bm{Z}(\bm{\eta})\bm{S}_{\text{rot}4}^1 \\ \bm{e}_3^T \frac{1}{m}\bm{B}(\bm{x})\bm{S}_{\text{tra}4}^1 \end{bmatrix} \begin{bmatrix} \omega_{M2}^2 \\ \omega_{M3}^2 \\ \omega_{M4}^2 \end{bmatrix} = \begin{bmatrix} \ddot{\theta} \\ \ddot{\phi} \\ \ddot{r}_3 + g \end{bmatrix}$$
* $\bm{S}_{\text{rot}4}^1$, $\bm{S}_{\text{tra}4}^1$ は、故障したモータ1に対応する1列目を削除した縮小行列です。
* この $3 \times 3$ 行列 $\bm{A}_3^1(\bm{\eta})$ が正則であれば、残存モータ速度 $\bm{u}_3^1$ を一意に決定して姿勢（ロール・ピッチ）と高度を制御し、墜落を回避することができます（ヨー軸は非制御となり、機体はスピンします）。
