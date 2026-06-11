<!-- GitHubとKaTeXの両方で \bm を有効にするマクロ定義 -->
$\newcommand{\bm}[1]{\boldsymbol{#1}}$
<!-- だが、GitHubでどうしても上手くいかないので、消して、手動変換するようにします。-->

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
>2. 上記定理の「一般化座標＋仮想仕事の原理＋静止座標系」を基準にした導出を、より単純な「ニュートン・オイラーの運動方程式＋動座標系」を基準にした導出のみで説明してみました。 $\boldsymbol{\omega}_{\dot{\boldsymbol{x}}}\rightarrow \boldsymbol{\Omega}_{\dot{\boldsymbol{x}}}$ 
>3. これによって、オイラー角のうちヨー角（$\psi$）のみを分離できることもはっきり分かりました。$\boldsymbol{\Omega}_{\dot{\boldsymbol{x}}} = \boldsymbol{\Omega}_{\dot{\boldsymbol{x}}}(\theta, \phi)$
>4. その過程で、定理の式を具体的な行列成分として書き出した。

---

## 3.1節：マルチロータの運動の数学的基礎（式 1〜8）

第3.1節では、ドローンの位置と姿勢を表現するための「静止（慣性）座標系」「動（機体）座標系」の定義、および座標変換のための数式を定義します。ここでは、理論的な基礎の準備をしています。さっと流していきましょう。

### ■ アフィン空間における作用と剛体の運動の定義（式 1, 2, 3）
$$a \mapsto a + \boldsymbol{v}, \quad a \in A^3, \; \boldsymbol{v} \in V \quad \text{--- (式 1)}$$
* **意味:** 3次元空間の幾何学的な「点」( $a$ )の集合であるアフィン空間 $A^3$ に対し、位置のズレ（相対ベクトル） $\boldsymbol{v}$ を表すベクトル空間 $V$ が平行移動として作用することを示しています。
* **物理的な位置づけ:** ドローンの移動を、点とベクトルの加算によって数学的に厳密に定義しています。
  * 静止座標系（慣性軸系）: $O + \text{span}\{\boldsymbol{e}_1, \boldsymbol{e}_2, \boldsymbol{e}_3\}$
  * 動座標系（質量中心 $O_c$ に固定）: $O_c + \text{span}\{\boldsymbol{E}_1, \boldsymbol{E}_2, \boldsymbol{E}_3\}$

この座標系において、剛体運動を定義します。並行移動と回転移動の組み合わせです。

$$\boldsymbol{T}(t): W \to w \quad \text{--- (式 2)}$$
$$\boldsymbol{T}(t) = \boldsymbol{C}(t)\boldsymbol{B}(t) \quad \text{--- (式 3)}$$

式(2)は、剛体（ドローン機体）の運動 $\boldsymbol{T}(t)$ は、動座標系 $W$ から静止座標系 $w$ へのアフィン写像として定義されます（写像の「定義域＝入力の集合 $W$ 」と「終域＝出力の集合 $w$ 」）。変換の向きに注意してください。動座標系のベクトルを静止座標系へと変換します。本論文では、一貫して動座標系→静止座標系が順方向です。この方が変換行列の表記として自然になります。

式(3)は、この運動が **「回転行列 $\boldsymbol{B}(t)$」** と **「並進（位置の移動）$\boldsymbol{C}(t)$」** の合成として一意に表せることを示しています。ここで、 $\boldsymbol{B}(t)\boldsymbol{C}(t)$ は、あたかも掛け算のように書かれていますが、実際には掛け算ではなく、「回転行列 $\boldsymbol{B}(t)$ による座標変換」と「並進ベクトル $\boldsymbol{C}(t)$ による位置の移動」の
**合成変換**（すなわちアフィン変換）を表しています。また、両者が時間 $t$ の関数になっていることにも注意してください。

### ■ 動径ベクトルと速度ベクトル（式 4, 5）

これを具体的な位置ベクトルと速度ベクトルの式に展開していきます。なお、一貫して小文字で静止座標系のベクトル（ $w$ の元）、大文字で動座標系のベクトル（ $W$ の元）を表す慣習が続きます。


$$\boldsymbol{q}(t) = \boldsymbol{r}(t) + \boldsymbol{B}(t)\boldsymbol{Q}(t) \quad \text{--- (式 4)}$$

$$
\dot{\boldsymbol{q}} = \dot{\boldsymbol{r}} + \frac{d}{dt}(\boldsymbol{B}\boldsymbol{Q}) = \dot{\boldsymbol{r}} + \dot{\boldsymbol{B}}\boldsymbol{Q} + \boldsymbol{B}\dot{\boldsymbol{Q}}\quad \text{--- (式 5)}
$$

式(4, 5)で、$\boldsymbol{q}, \boldsymbol{r} \in w$ は、静止座標系における位置ベクトルであり、$\boldsymbol{Q} \in W$ は、動座標系における位置ベクトル（動径ベクトルと呼んでいる）を表しています。

* **式(4)の意味:** 静止座標系から見た動点の位置 $\boldsymbol{q}(t) \in w$ は、「重心の位置 $\boldsymbol{r}(t) \in w$」に「機体内の相対位置 $\boldsymbol{Q}(t) \in W$ を回転行列 $\boldsymbol{B}(t)$ で静止座標系へ変換したベクトル $\boldsymbol{B}(t)\boldsymbol{Q}(t) \in w$ 」を加えたものです。
* **式(5)の意味:** これを時間微分した速度式です。機体が変形しない剛体であれば、機体内での相対位置の変化はないため、第3項 $\boldsymbol{B}\dot{\boldsymbol{Q}} = \boldsymbol{0}$ となります。

この(式4,5)から、非常に汎用的な運動方程式(式13)が後に導出されます。さらに準備として、ドローンの姿勢を表すテイト-ブライアン角（ヨー、ピッチ、ロール）に対応する基本回転行列を定義します。

### ■ テイト-ブライアン角に対応する基本回転行列（式 6, 7, 8, 9）

ドローンの姿勢（回転）は、ヨー（$\psi$）、ピッチ（$\theta$）、ロール（$\phi$）の3つの軸まわりの回転を、この順に静止座標系の基底を回転して動座標系の基底に重ねることで表現します（テイト・ブライアン角）。3つの角の順番にはこの定義以外様々な流儀（12種類）があるので注意してください。論文での定義は以下の通りです：

| ヨー回転 $\boldsymbol{R}_{\psi}$<br>（Z軸まわり、式 6） | ピッチ回転 $\boldsymbol{R}_{\theta}$<br>（中間Y軸まわり、式 7） | ロール回転 $\boldsymbol{R}_{\phi}$<br>（機体X軸まわり、式 8） |
| :---: | :---: | :---: |
| $\boldsymbol{R}_{\psi} = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}$ | $\boldsymbol{R}_{\theta} = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}$ | $\boldsymbol{R}_{\phi} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix}$ |

注：これは箱庭physicsで採用されているオイラー角と同じ順番、同じ記号の右手形です。ただし箱庭はz軸が下向きである点が異なります。
箱庭physics(https://github.com/toppers/hakoniwa-px4sim/blob/main/drone_physics/README-ja.md)では、
- 機体座標系 body frame は FRD(front-right-down)。
- 地上座標系 ground frame は NED(north-east-down)。

理論的には重力方向の扱いと制御時のラダー・エレベータ・エルロンの符号が異なりますが、以下の論文と基本的には箱庭physicsとそのまま同じ式が成り立ちます。

また、ドローンの姿勢を表す回転行列 $\boldsymbol{B}$ は、これらの基本回転行列を掛け合わせることで構成されます（式 9）。この合成回転行列 $\boldsymbol{B}$ を用いて、機体固定座標系から静止座標系への変換を行います。

>[!IMPORTANT]
>$$\boldsymbol{B}(\psi, \theta, \phi) = \boldsymbol{R}_{\psi}\boldsymbol{R}_{\theta}\boldsymbol{R}_{\phi} \quad \text{--- (式 9)}$$

回転行列 $\boldsymbol{B}$ について、少し詳しくまとめておきます（以下は飛ばしても大丈夫）。

> [!NOTE]
> **回転変換行列 $B$**
>上記箱庭physicsに計算が書かれていますが、このように３つの回転をこの順にかけることで、静止座標系から動座標系へと基底の取り替え行列としての $\boldsymbol{B}$ が定義されます。
>$$
>\begin{bmatrix}
>\boldsymbol{E}_1 & \boldsymbol{E}_2 & \boldsymbol{E}_3
>\end{bmatrix}
>=\begin{bmatrix}
>\boldsymbol{e}_1 & \boldsymbol{e}_2 & \boldsymbol{e}_3
>\end{bmatrix}
>\boldsymbol{B}
>$$
>
>このように見ると、例えば $\boldsymbol{B}$ の一列目は $\boldsymbol{E}_1$ を静止座標系で表現したときの成分表示になっていることが見て取れるでしょう。
>
>さらに、ベクトルの普遍性（「基底」と「座標」の掛け算が変化しない）より、
>$$
>\begin{bmatrix}
>\boldsymbol{e}_1 & \boldsymbol{e}_2 & \boldsymbol{e}_3
>\end{bmatrix}
>\begin{bmatrix}
>x \\ y \\ z
>\end{bmatrix}
>=\begin{bmatrix}
>\boldsymbol{E}_1 & \boldsymbol{E}_2 & \boldsymbol{E}_3
>\end{bmatrix}
>\begin{bmatrix}
>x' \\ y' \\ z'
>\end{bmatrix}
>$$
>よって、それぞれの基底を使った座標表現の変換は以下のようになります。
>$$
>\begin{bmatrix}x \\ y \\ z\end{bmatrix}
>= \boldsymbol{B}
>\begin{bmatrix}x' \\ y' \\ z'\end{bmatrix} \quad \text{すなわち} \quad
>\begin{bmatrix}
>静\\止\\座\\標
>\end{bmatrix}
>= \boldsymbol{B}
>\begin{bmatrix}
>動\\座\\標
>\end{bmatrix}
>$$
>これは以降でもとてもよく使われる、静止座標と動座標の変換公式です。どんなベクトル $a$ についてもこれが成り立ちます。例えば速度ベクトル、各速度ベクトル、力ベクトル、トルクベクトルなどです。
>
>いくつか覚えておくべき性質を上げます。
>- $\boldsymbol{B}$ は直交行列である。すなわち、$\boldsymbol{B}\boldsymbol{B}^T=\boldsymbol{B}^T\boldsymbol{B}=I$ (単位行列)
>- $\boldsymbol{B}$ はある１つの軸周りの回転として定義できる。その軸は $\boldsymbol{B}$ の固有値1に対応する固有ベクトル（動座標系）であるこのベクトルは、変換の前後で変化しない。
>- 任意の動座標ベクトル $\boldsymbol{A}$ に対して、静止座標系での表現は $\boldsymbol{a} = \boldsymbol{B}\boldsymbol{A}$ となる。逆に、静止座標系のベクトル $\boldsymbol{a}$ を動座標系で表すときは $\boldsymbol{A} = \boldsymbol{B}^T\boldsymbol{a}$ となる。
>- $\boldsymbol{B}$ は回転の構造を保つ。すなわち、ベクトルの外積をとってから回転しても、ベクトルをあらかじめ回転させてから外積をとっても同じ結果になる。 $\boldsymbol{B}(\boldsymbol{a} \times \boldsymbol{b}) = (\boldsymbol{B}\boldsymbol{a}) \times (\boldsymbol{B}\boldsymbol{b})$
>- $\boldsymbol{B}$ はDCM（Direction Cosine Matrix）とも呼ばれる。回転行列の各列は、動座標系の基底ベクトルを静止座標系で表現したときの成分表示になっている。
>- $\boldsymbol{B}$ の時間微分 $\dot{\boldsymbol{B}}$ を左から動座標系のベクトルに掛けることは、回転の角速度ベクトル $\boldsymbol{\Omega}$ を外積で掛けることと等価である。すなわち、 $\dot{\boldsymbol{B}}\boldsymbol{A} = \boldsymbol{B}(\boldsymbol{\Omega} \times \boldsymbol{A})$ となる（ポアソンの定理、後述）。

もっとも覚えておくべきは、以下の変換です。静止座標系と動座標系の間の基本的な変換式になります。ベクトル量であれば、この変換は常に成り立ちます（オイラー角やその時間微分には成り立たない）。

>[!IMPORTANT]
>| 静止座標系 $\{\boldsymbol{e}_1, \boldsymbol{e}_2, \boldsymbol{e}_3\}$ での座標 | 変換 | 動座標系 $\{\boldsymbol{E}_1, \boldsymbol{E}_2, \boldsymbol{E}_3\}$ での座標 |
>| :---: | :---: | :---: |
>| <br>速度 $\boldsymbol{v}$ <br>角速度 $\boldsymbol{\omega}$ <br>その他ベクトル<br>（力、トルク、運動量、角運動量） |  $\boldsymbol{B}^T$ を掛ける<br>$\rightarrow$<br>$\leftarrow$<br>$\boldsymbol{B}$ を掛ける   | <br>速度 $\boldsymbol{V}$<br> 角速度 $\boldsymbol{\Omega}$<br>その他ベクトル<br>（力、トルク、運動量、角運動量） |

### ■ 回転座標系における速度と加速度の展開（式 10, 11, 12, 13）

座標系が回転している環境下での、位置ベクトルの微分（速度・加速度）の展開式です。本論文では、ベクトル積（外積）をブラケット記法 $[\boldsymbol{a}, \boldsymbol{b}] = \boldsymbol{a} \times \boldsymbol{b}$ で表現しています。

* **速度ベクトル（式 10）：**
  機体が角速度 $\boldsymbol{\Omega}$ (動座標系) または $\boldsymbol{\omega}$ (静止座標系) で回転しているとき、動径ベクトル $\boldsymbol{Q}$ の微分は次のようになります。
  $$\dot{\boldsymbol{q}} = \dot{\boldsymbol{r}} + \frac{d}{dt}(\boldsymbol{B}\boldsymbol{Q}) = \dot{\boldsymbol{r}} + \dot{\boldsymbol{B}}\boldsymbol{Q} + \boldsymbol{B}\dot{\boldsymbol{Q}} = \dot{\boldsymbol{r}} + \boldsymbol{B} [\boldsymbol{\Omega}, \boldsymbol{Q}] + \boldsymbol{B}\dot{\boldsymbol{Q}} \quad \text{--- (式 5)再掲}$$

機体内固定の点であれば $\dot{\boldsymbol{Q}} = \boldsymbol{0}$ となり、論文中の
$$
\dot{\boldsymbol{q}} = \dot{\boldsymbol{r}} + [\boldsymbol{\omega}, \boldsymbol{B}\boldsymbol{Q}] = \dot{\boldsymbol{r}} + \boldsymbol{B}[\boldsymbol{\Omega}, \boldsymbol{Q}] = \dot{\boldsymbol{r}} + [\boldsymbol{B}\boldsymbol{\Omega}, \boldsymbol{B}\boldsymbol{Q}] \quad \text{--- (式 10) $\dot{Q} = 0$ の場合}
$$
 と等価になります。また、 $\boldsymbol{B} [\boldsymbol{\Omega}, \boldsymbol{Q}] = [\boldsymbol{B}\boldsymbol{\Omega}, \boldsymbol{B}\boldsymbol{Q}]$ となる理由は、回転行列 $\boldsymbol{B}$ は外積の構造を保つためです。すなわち、2つのベクトルの外積をとってから回転しても、2つのベクトルをあらかじめ回転させてからその外積をとっても同じ結果になります。この(式10)も今後登場しませんのでなるほど、と読んでおいて大丈夫。(式5)はさらにこのあと加速度の式(式13)に展開されますので、そちらで詳しく解説します。

### ■ 角速度ベクトルとオイラー角の微分（式 11, 12）
$$\boldsymbol{\Omega} = \boldsymbol{B}^T \boldsymbol{\omega} \quad \text{--- (式 11)}$$
$$\boldsymbol{\omega} = \dot{\psi}\boldsymbol{e}_3 + \dot{\theta}\boldsymbol{E}_2^{(-2)} + \dot{\phi}\boldsymbol{E}_1^{(-1)} \quad \text{--- (式 12)}$$
* **意味:** 機体固定の瞬間角速度 $\boldsymbol{\Omega}$ と角速度 $\boldsymbol{\omega}$ の関係式、およびオイラー角の微分（$\dot{\psi}, \dot{\theta}, \dot{\phi}$）が角速度 $\boldsymbol{\omega}$ とどう結びついているかを示します。

(式12)は、この後出てこないので、各オイラー角の時間微分が変換途中の座標軸に対してどのように速度で構成されることに注意するだけでよいです。
(式11)は上の「IMPORTANT」で紹介しましたし、この後何度も出てきます。逆方向の変換と合わせて覚えておきます。動系から静止系へのベクトルの変換は、 $\boldsymbol{B}$ を掛けることで行われることを思い出しまししょう。各速度は「ベクトル」ですので、この規則が当てはまります（オイラー角には当てはまりません、オイラー角は「ベクトル」でなく、成分ごとの和やスカラー倍したベクトルは物理的に意味をなしません）。

$$\boldsymbol{\Omega} = \boldsymbol{B}^T \boldsymbol{\omega} \quad \text{--- (式 11)}\\
\boldsymbol{\omega} = \boldsymbol{B} \boldsymbol{\Omega} \quad \text{--- (式 11)'}$$

を構成するかを示す式です。これを式11の関係式と組み合わせることで、オイラー角の微分から角速度への変換行列 $\boldsymbol{\omega}_{\dot{\boldsymbol{x}}}(\psi, \theta)$ を導出することができます。
  
* **加速度ベクトル（式 13）：**
速度ベクトル（式10）をさらにもう一度時間微分することで、回転系における加速度方程式を得ます。

$$\ddot{\boldsymbol{q}} = \ddot{\boldsymbol{r}} + \boldsymbol{B} [\boldsymbol{\Omega}, [\boldsymbol{\Omega}, \boldsymbol{Q}]] + \boldsymbol{B} [\dot{\boldsymbol{\Omega}}, \boldsymbol{Q}] + 2\boldsymbol{B} [\boldsymbol{\Omega}, \dot{\boldsymbol{Q}}] + \boldsymbol{B}\ddot{\boldsymbol{Q}} \quad \text{--- (動座標系角速度ベクトル)} \boldsymbol{\Omega} \\
\ddot{\boldsymbol{q}} = \ddot{\boldsymbol{r}} + [\boldsymbol{\omega}, [\boldsymbol{\omega}, \boldsymbol{B}\boldsymbol{Q}]] + [\dot{\boldsymbol{\omega}}, \boldsymbol{B}\boldsymbol{Q}] + 2[\boldsymbol{\omega}, \boldsymbol{B}\dot{\boldsymbol{Q}}] + \boldsymbol{B}\ddot{\boldsymbol{Q}} \quad \text{--- (静止座標系角速度ベクトル)}\boldsymbol{\omega}$$

  これで、回転座標系における加速度の展開式が得られます（この計算詳細は後述します）。

#### 💡 各項の物理的な意味
1. **$\ddot{\boldsymbol{r}}$ : 重心の並進加速度**
   * 静止座標系に対するドローン重心全体の加速度です。
2. **$\boldsymbol{B} [\boldsymbol{\Omega}, [\boldsymbol{\Omega}, \boldsymbol{Q}]]$ または $[\boldsymbol{\omega}, [\boldsymbol{\omega}, \boldsymbol{B}\boldsymbol{Q}]]$ : 遠心力**
   * 回転運動によって、回転中心から外側へ向かって引き離すように作用する慣性加速度です。
3. **$\boldsymbol{B} [\dot{\boldsymbol{\Omega}}, \boldsymbol{Q}]$ または $[\dot{\boldsymbol{\omega}}, \boldsymbol{B}\boldsymbol{Q}]$ : 回転慣性力**
   * 回転の角速度が変化（角加速度 $\dot{\boldsymbol{\Omega}}$ または $\dot{\boldsymbol{\omega}}$ が発生）したときに、回転の接線方向に発生する加速度です。
4. **$2\boldsymbol{B} [\boldsymbol{\Omega}, \dot{\boldsymbol{Q}}]$ または $2[\boldsymbol{\omega}, \boldsymbol{B}\dot{\boldsymbol{Q}}]$ : Colioris力**
   * 回転している座標系の中で、対象の点が相対速度 $\dot{\boldsymbol{Q}}$ を持って移動する際に、回転軸と速度の双方に直交する方向に受ける見かけの加速度です。
5. **$\boldsymbol{B}\ddot{\boldsymbol{Q}}$ : 相対加速度**
   * ドローンの機体固定座標系から見た、動点の純粋な相対加速度です。

難しくなりましたが、この汎用式（式13）は、この論文の最後まで（剛体のみを扱うため）活用されません。一度眺めて貰えばよいでしょう。
すなわち、本論文では $\dot{\boldsymbol{Q}} = \boldsymbol{0}, \ddot{\boldsymbol{Q}} = \boldsymbol{0}$ ため、4.のColioris力の項、5.の相対加速度の項は消えます。さらに、$\boldsymbol{r}$ をドローンの重心位置としているため（ $\Sigma m_i \boldsymbol{Q}_i = 0$ ）、 $\boldsymbol{Q}$ がそのまま関与する 2.の遠心力、3.のオイラー加速度の項も釣り合って消えます。すると、実際に本論文で使われる重心の並進に関する式は、

$$\ddot{\boldsymbol{q}} = \ddot{\boldsymbol{r}} \quad \text{--- (式 13)’}$$

となるだけです。動座標系 $\dot{\boldsymbol{Q}}$ に起因する項は消えます。

ただし、ここから、動座標系での並進の運動方程式を立式する際には、やはり見かけの力が（再度）発生します。
重心の速度 $\boldsymbol{v} = \dot{\boldsymbol{r}}$ とし、

$$
\boldsymbol{f} = m\dot{\boldsymbol{v}} \quad \text{--- (*1)ニュートンの方程式(静止座標系)}
$$

という重心に対する静止座標でのニュートンの方程式を立式します（ $\boldsymbol{f} \in w$ ）。
重心の速度 $\boldsymbol{v} = \dot{\boldsymbol{r}} \in w$ を動座標系で表して $\boldsymbol{V} \in W$ とすると $\boldsymbol{v} = \boldsymbol{B}\boldsymbol{V}$ となるため、輸送定理（すぐ後に解説）により、（式13'）は、

$$\ddot{\boldsymbol{r}} = \dot{\boldsymbol{v}} = \boldsymbol{B} (\dot{\boldsymbol{V}} + \boldsymbol{\Omega} \times \boldsymbol{V})$$

となり、重心についても動座標系で立式した場合のベクトルには慣性力がかかります。(*)は、

$$
\boldsymbol{f} = m \dot{\boldsymbol{v}} = m \boldsymbol{B} (\dot{\boldsymbol{V}} + \boldsymbol{\Omega} \times \boldsymbol{V})
$$
$\boldsymbol{F} = \boldsymbol{B}^T \boldsymbol{f} \in W$ として両辺に $\boldsymbol{B}^T$ を掛けて動座標系で表すと、
$$
\boldsymbol{F} = m\dot{\boldsymbol{V}} + m \boldsymbol{\Omega} \times \boldsymbol{V}\quad \text{--- (*1')ニュートンの方程式(動座標系)}
$$

となり、コリオリ力に相当する項が再度現れます。原点（重心）の並進に対してかかるコリオリ力です。一旦、剛体条件で消えたのですが、並進の中にも含まれていたのですね。そして、この式が、ドローンの運動方程式の基礎となり、後の状態方程式の導出に繋がっていきます。


>[!NOTE] ポアソンの関係式と輸送定理
>回転行列 $\boldsymbol{B}(t)$、その動座標系での回転角速度ベクトルを $\boldsymbol{\Omega} \in W$ 、対応する静止座標系での回転各速度ベクトルを $\boldsymbol{\omega} = \boldsymbol{B}\boldsymbol{\Omega} \in w$ とすると、任意の動座標ベクトル  $\boldsymbol{Q} \in W$ 、その対応する静止座標ベクトル $\boldsymbol{q} = \boldsymbol{B}\boldsymbol{Q} \in w$ に対して、
>**ポアソンの関係式** ：
>$$
>\begin{align*}
>\dot{\boldsymbol{B}}\boldsymbol{Q} &= \boldsymbol{B} (\boldsymbol{\Omega} \times \boldsymbol{Q})\\
> &= \boldsymbol{\omega} \times (\boldsymbol{B}\boldsymbol{Q})\\
> &= \boldsymbol{\omega} \times \boldsymbol{q}
>\end{align*}
>$$
>
>**輸送定理** ：（ポアソンの関係式を使って容易に証明可）
>$$
>\dot{\boldsymbol{q}} = \boldsymbol{B} (\boldsymbol{\Omega} \times \boldsymbol{Q} + \dot{\boldsymbol{Q}})
>$$
>
>すなわち、「静止座標での時間微分を動座標の時間微分に変換する際、 $\boldsymbol{\Omega}$ の外積を掛けたものをプラスする」。
>
>**直感的理解** ：
>動座標系がZ軸周りに回転していた場合、動座標系の+x軸方向に動いている点は、静止座標系から見ると、回転の影響で+y軸方向にも動いているように見えることがわかります。これが $\boldsymbol{\Omega} \times \boldsymbol{Q}$ です。

これらを使って、

#### 💡 式 13 の数学的導出メモ

ここでは、(式5)の前半式から輸送定理を用いて(式13)導出してみます。
$$
\begin{align*}
\dot{\boldsymbol{q}} &= \dot{\boldsymbol{r}} + \frac{d}{dt}(\boldsymbol{B}\boldsymbol{Q}) \quad \text{--- (式 5)再掲} \\
&= \dot{\boldsymbol{r}} + \boldsymbol{B}(\Omega \times \boldsymbol{Q} + \dot{\boldsymbol{Q}}) \quad \because \text{輸送定理}
\end{align*}
$$
さらに時間微分、ポアソンの関係式を使って展開していきます。
$$
\begin{align*}
\ddot{\boldsymbol{q}} &= \ddot{\boldsymbol{r}} + \dot{\boldsymbol{B}}(\Omega \times \boldsymbol{Q} + \dot{\boldsymbol{Q}}) + \boldsymbol{B} \frac{d}{dt}(\Omega \times \boldsymbol{Q} + \dot{\boldsymbol{Q}}) \\
&= \ddot{\boldsymbol{r}} + \boldsymbol{B}(\Omega \times (\Omega \times \boldsymbol{Q})) + \boldsymbol{B}(\Omega \times \dot{\boldsymbol{Q}}) + \boldsymbol{B} \frac{d}{dt}(\Omega \times \boldsymbol{Q}) + \boldsymbol{B}\ddot{\boldsymbol{Q}} \\
&= \ddot{\boldsymbol{r}} + \boldsymbol{B}(\Omega \times (\Omega \times \boldsymbol{Q})) + 2\boldsymbol{B}(\Omega \times \dot{\boldsymbol{Q}}) + \boldsymbol{B}(\dot{\Omega} \times \boldsymbol{Q}) + \boldsymbol{B}\ddot{\boldsymbol{Q}}
\end{align*}
$$

再度、論文の式(13)を確認してみましょう。

$$
\begin{align*}
\ddot{\boldsymbol{q}} &= \ddot{\boldsymbol{r}} + \boldsymbol{B}\ddot{\boldsymbol{Q}} + 2\dot{\boldsymbol{B}}\dot{\boldsymbol{Q}} + \ddot{\boldsymbol{B}}\boldsymbol{Q} \quad \text{--- 式13}\\
&= \ddot{\boldsymbol{r}} + \boldsymbol{B} [\boldsymbol{\Omega}, [\boldsymbol{\Omega}, \boldsymbol{Q}]] + \boldsymbol{B} [\dot{\boldsymbol{\Omega}}, \boldsymbol{Q}] + 2\boldsymbol{B} [\boldsymbol{\Omega}, \dot{\boldsymbol{Q}}] + \boldsymbol{B}\ddot{\boldsymbol{Q}} \quad \text{--- (動座標系角速度ベクトル)} \boldsymbol{\Omega}\\
&= \ddot{\boldsymbol{r}} + [\boldsymbol{\omega}, [\boldsymbol{\omega}, \boldsymbol{B}\boldsymbol{Q}]] + [\dot{\boldsymbol{\omega}}, \boldsymbol{B}\boldsymbol{Q}] + 2[\boldsymbol{\omega}, \boldsymbol{B}\dot{\boldsymbol{Q}}] + \boldsymbol{B}\ddot{\boldsymbol{Q}} \quad \text{--- (静止座標系角速度ベクトル)}\boldsymbol{\omega}
\end{align*}
$$

外積の記号は違いますが、同じ式が得られました。 $B [\Omega, Q] = [B\Omega, BQ] （外積の保存）= [\omega, BQ]$ から二つの式は同じにになります。これで、(式13)の導出が確認できました（ただし、この式は今後登場することはありません。見かけの力とその原因を洗い出した、という意味があります）。


---

## 3.2節：剛体の回転による方程式（オイラーの方程式）（式 14〜18）

第3.2節では、ドローンの姿勢変化と力の伝達、およびそれらを統一した非線形の状態方程式（運動方程式）を組み立てます。

### ■ オイラーの運動方程式（式 14, 15）
$$
\begin{align*}
\boldsymbol{h} &= \boldsymbol{I}\boldsymbol{\omega} \quad \text{--- (式 14)、角運動量はモーメントと角速度の積} \\
\dot{\boldsymbol{h}} &= \boldsymbol{\tau} \quad \text{--- (式 15)、トルクが角運動量の時間変化を生む(*2) オイラーの方程式（静止座標）}
\end{align*}
$$
* **意味:** 静止座標系における角運動量 $\boldsymbol{h}$ とトルク $\boldsymbol{\tau}$ の関係です。

静止座標系では、慣性モーメント $\boldsymbol{I} = \boldsymbol{I}(\boldsymbol{x})$ はオイラー角 $\boldsymbol{x} = (\psi, \theta, \phi)^T$ に依存する扱いにくい量になります。よって、重心を原点とする動座標系で扱うことで、 $\hat{\boldsymbol{I}}$ という姿勢に依存しない主軸対角行列に変換できます。$\hat{\boldsymbol{I}}$ はドローンの質量分布に依存する定数行列であり、ドローンの形状や質量配置が変わらない限り一定です。ちなみに、$\boldsymbol{I}$ と $\hat{\boldsymbol{I}}$ は、回転行列 $\boldsymbol{B}$ を用いて $\boldsymbol{I} = \boldsymbol{B}\hat{\boldsymbol{I}}\boldsymbol{B}^T$ という関係で結びついています。

(式15)の両辺を動座標系で記述します。 $\boldsymbol{H}, \boldsymbol{\Omega}, \boldsymbol{T} \in W$ を動座標系での角運動量、角速度、トルクとすると、これらはベクトルの変換規則に従って、
$$
\begin{align*}
\boldsymbol{h} &= \boldsymbol{B}\boldsymbol{H}\\
\boldsymbol{\tau} &= \boldsymbol{B} \boldsymbol{T}\\
\boldsymbol{\omega} &= \boldsymbol{B}\boldsymbol{\Omega}
\end{align*}
$$
 であり、(式14)にそれぞれ代入して時間微分を計算すると、輸送定理より
が成り立ち、これが(式15)よりトルクになるため、
$$\boldsymbol{B}(\boldsymbol{\Omega} \times \boldsymbol{H} + \dot{\boldsymbol{H}}) = \boldsymbol{B} \boldsymbol{T}$$

が得られます。両辺に $\boldsymbol{B}^T$ を掛けて、

$$\boldsymbol{\Omega} \times \boldsymbol{H} + \dot{\boldsymbol{H}} = \boldsymbol{T}\quad \text{--- (式 15)の動座標系版}$$

さらに、(式16) $\boldsymbol{H} = \hat{\boldsymbol{I}}\boldsymbol{\Omega}, \dot{\boldsymbol{H}} = \hat{\boldsymbol{I}}\dot{\boldsymbol{\Omega}}$ として、オイラーの運動方程式が得られます。これら２つの方程式で、剛体ドローンの姿勢変化と速度に対する力とトルクの関係は、動座標系で完全に表現できます。繰り返しですが、動座標系にするのは、慣性モーメントを姿勢によらない定数として対角化したいためです。

>[!IMPORTANT]
>**剛体の運動方程式（動座標系）**
>$$
>\begin{align*}
>\boldsymbol{F} &= m\dot{\boldsymbol{V}} + m \boldsymbol{\Omega} \times \boldsymbol{V}\quad \text{--- (*1')ニュートンの方程式}\\
>\boldsymbol{T} &= \hat{\boldsymbol{I}}\dot{\boldsymbol{\Omega}} + \boldsymbol{\Omega} \times (\hat{\boldsymbol{I}}\boldsymbol{\Omega}) \quad \text{---(*2') オイラーの方程式（(式 14,15)の動座標系版）}
>\end{align*}
>$$

### ■ ドローンのオイラー角空間（式 19〜21）
$span\{\boldsymbol{\epsilon}_1, \boldsymbol{\epsilon}_2, \boldsymbol{\epsilon}_3\}$ をドローンのオイラー角空間と定義し、オイラー角 $\psi, \theta, \phi$ をこの空間の座標として定義します。これにより、ドローンの姿勢を表す状態変数 $\boldsymbol{x}$ を以下のように定義できます。
$$
\begin{align*}
\boldsymbol{x} &= \psi \boldsymbol{\epsilon}_1 + \theta \boldsymbol{\epsilon}_2 + \phi \boldsymbol{\epsilon}_3 = \psi \boldsymbol{\epsilon}_1 + \boldsymbol{\eta} \quad \text{--- (式 19)}\\
\dot{\boldsymbol{x}} &= \dot{\psi} \boldsymbol{\epsilon}_1 + \dot{\theta} \boldsymbol{\epsilon}_2 + \dot{\phi} \boldsymbol{\epsilon}_3 = \dot{\psi} \boldsymbol{\epsilon}_1 + \dot{\boldsymbol{\eta}} \quad \text{--- (式 20)}
\end{align*}
$$
ここで、 $\psi$ はドローンのヨー角で、この角だけ分離した扱いになっています。これは後に、モーター故障の際にこの角だけ制御を切り離して他の角を制御する、という戦略を見据えた定式化です。 $\boldsymbol{\eta} = \theta \boldsymbol{\epsilon}_2 + \phi \boldsymbol{\epsilon}_3$ は、ピッチとロールの角度をまとめたベクトルです。

さらに、この3つの状態変数とその微分、すなわち $(\boldsymbol{x}, \dot{\boldsymbol{x}}) \in \mathbb{R}^6$ という6変数を用いて状態空間とします。




### ■ ラグランジアン $L$ と一般化力（式 22, 23）

ここで、ラグランジアンが唐突に登場しますが、やろうとしていることは先の姿勢（オイラー角）に関する状態空間に対応する、

- 一般化座標（ $\boldsymbol{x} = (\psi, \theta, \phi)$ ）
- 一般化速度（ $\dot{\boldsymbol{x}} = (\dot{\psi}, \dot{\theta}, \dot{\phi}) $ ）
- 対応する一般化力（ $\boldsymbol{f} = \boldsymbol{B}(\boldsymbol{x})\boldsymbol{F}_{\text{rot}}(\boldsymbol{u})$ ）

を定義して、ニュートン・オイラー方程式を一般化座標系で表現することです。そして $\boldsymbol{F}_{\text{rot}}(\boldsymbol{u})$ は、ドローンのモーターから発生する回転トルクを表すベクトルで、制御入力 $\boldsymbol{u}$ に依存します。これらから、
>[!IMPORTANT]
>入力 $\boldsymbol{u}$ （モーターへの制御信号）に対応する出力 $\ddot{\boldsymbol{x}}$ （状態空間内の状態ベクトルの遷移）の「感度」を導出することが目的

です。これによって、入力 $\boldsymbol{u}$ に対するドローンの姿勢をオイラー角空間で制御できるようになります。

では、ラグランジアンの式を解説しますが、ここの理解を飛ばしても問題ありません。通常のニュートン・オイラー方程式からも、同様の結果が導かれます。

$$L = \frac{1}{2} m \langle \dot{\boldsymbol{r}}, \dot{\boldsymbol{r}} \rangle + \frac{1}{2} \langle \hat{\boldsymbol{I}}\boldsymbol{\Omega}, \boldsymbol{\Omega} \rangle - mg \langle \boldsymbol{r}, \boldsymbol{e}_3 \rangle \quad \text{--- (式 22)}\\
\langle \boldsymbol{B}(\boldsymbol{x})\boldsymbol{F}_{\text{rot}}(\boldsymbol{u}), \delta\boldsymbol{\omega} \rangle = \langle \boldsymbol{\omega}_{\dot{\boldsymbol{x}}}(\psi, \theta)^T \boldsymbol{B}(\boldsymbol{x})\boldsymbol{F}_{\text{rot}}(\boldsymbol{u}), \delta\dot{\boldsymbol{x}} \rangle \quad \text{--- (式 23)}$$
* **第22式:** 並進の運動エネルギー、回転の運動エネルギー、重力ポテンシャルから構成されるラグランジアンです。
* **第23式:** 仮想仕事の原理（の時間微分）に基づき、モータによるモーメント $\boldsymbol{F}_{\text{rot}}$ をオイラー角 $\boldsymbol{x}$ の一般化力へ変換します。

* **変換行列 $\boldsymbol{\omega}_{\dot{\boldsymbol{x}}} = \boldsymbol{\omega}_{\dot{\boldsymbol{x}}}(\psi, \theta)$（式 28）：**
$$
\boldsymbol{\omega}_{\dot{\boldsymbol{x}}} = \boldsymbol{\omega}_{\dot{\boldsymbol{x}}}(\psi, \theta) = \begin{bmatrix} 0 & -\sin\psi & \cos\theta \cos\psi \\ 0 & \cos\psi & \cos\theta \sin\psi \\ 1 & 0 & -\sin\theta \end{bmatrix} \quad \text{--- (式 28)}
$$

この(式28)がポイントになります。この式は単なる計算のようでありますが深い意味を持ちます。



- **オイラー角の微分 $\dot{\boldsymbol{x}}$ と角速度 $\boldsymbol{\omega}$ の関係を表す変換行列** です。実は、角速度 $\boldsymbol{\omega}$ はオイラー角の微分 $\dot{\boldsymbol{x}}$ の **線形変換** として表すことができます。その変換の表現行列が $\boldsymbol{\omega}_{\dot{\boldsymbol{x}}}$ です。$\boldsymbol{\omega} = \boldsymbol{\omega}_{\dot{\boldsymbol{x}}}(\psi, \theta) \dot{\boldsymbol{x}}$ 、すなわち（3次元ベクトル）＝（3x3行列） $\times$ （3成分列）という関係が成り立ちます。
- そして、その **変換行列** $\boldsymbol{\omega}_{\dot{\boldsymbol{x}}}$ は変数 **$(\psi, \theta)$ のみ** で決まります。
- さらに、（明示的に論文に記述がないですが）これを動座標系で計算した、$\boldsymbol{\Omega} = \boldsymbol{B}^T \boldsymbol{\omega} = \boldsymbol{B}^T \boldsymbol{\omega}_{\dot{\boldsymbol{x}}}(\psi, \theta) \dot{\boldsymbol{x}}$ を計算すると、今度は、

$$
\begin{align*}
\boldsymbol{\Omega}_{\dot{\boldsymbol{x}}} &= \boldsymbol{\Omega}_{\dot{\boldsymbol{x}}}(\theta, \phi) \quad (= \boldsymbol{B}^T \boldsymbol{\omega}_{\dot{\boldsymbol{x}}}(\psi, \theta)) \quad \text{--- (式 28)の動座標系版}\\
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
\boldsymbol{\Omega} &= \boldsymbol{\Omega}_{\dot{\boldsymbol{x}}}(\theta, \phi) \dot{\boldsymbol{x}}\\
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

（検算として、実際に $\boldsymbol{\Omega} = \boldsymbol{B}^T \boldsymbol{\omega}_{\dot{\boldsymbol{x}}}(\psi, \theta)$ を行列計算してみると、 

$$
\begin{align*}
\boldsymbol{B}^T \boldsymbol{\omega}_{\dot{\boldsymbol{x}}}
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
\; (= \boldsymbol{\Omega}_{\dot{\boldsymbol{x}}}(\theta,\phi))
\end{align*}
$$
検算終わり）

この計算から、ヨー角（$\psi$）を式から追払うことができ、動座標系での角速度 $\boldsymbol{\Omega}$ は、オイラー角の時間変化に対して線形で、ピッチとロールの角度（$\theta, \phi$）のみに依存していることになります。これによって、ヨー角を制御から独立させることができるのです。この逆行列を用いることで、動座標系の角速度 $\boldsymbol{\Omega}$ からオイラー角の時間微分 $\dot{\boldsymbol{x}}$ に変換することもできます。すなわち、 $\boldsymbol{\omega}_{\dot{\boldsymbol{x}}}, \boldsymbol{\Omega}_{\dot{\boldsymbol{x}}}$ の逆行列をそれぞれ計算して、以下のようになります。

$$
\begin{align*}
\dot{\boldsymbol{x}} &= \boldsymbol{\omega}_{\dot{\boldsymbol{x}}}(\psi, \theta)^{-1} \boldsymbol{\omega}\\
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
\dot{\boldsymbol{x}} &= \boldsymbol{\Omega}_{\dot{\boldsymbol{x}}}(\theta, \phi)^{-1} \boldsymbol{\Omega}\\
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

これにより、求めるオイラー角の変化率を角速度から得ることができます。さらにそれを微分することで、オイラー角の加速度 $\ddot{\boldsymbol{x}}$ も角加速度 $\dot{\boldsymbol{\omega}}$ から求められます。これが、感度（以下の章で登場する $\boldsymbol{Z}(\boldsymbol{\eta})$ ）の肝になります。


### ■ 回転の明示的な状態方程式（定理 1 / 式 29〜32）
$$\frac{d}{dt} \begin{bmatrix} \boldsymbol{x} \\ \dot{\boldsymbol{x}} \end{bmatrix} = \begin{bmatrix} \dot{\boldsymbol{x}} \\ \boldsymbol{Y}(\boldsymbol{\eta}, \dot{\boldsymbol{x}}) + \boldsymbol{Z}(\boldsymbol{\eta})\boldsymbol{F}_{\text{rot}}(\boldsymbol{u}) \end{bmatrix} \quad \text{--- (式 30)}$$

* **制御入力ゲイン行列 $\boldsymbol{Z}(\boldsymbol{\eta})$（式 31）:**
  $$\boldsymbol{Z}(\boldsymbol{\eta}) = (\boldsymbol{B}\hat{\boldsymbol{I}}\boldsymbol{B}^T \cdot \boldsymbol{\omega}_{\dot{\boldsymbol{x}}})^{-1} \boldsymbol{B} \quad \text{--- (式 31)}$$
 * **慣性項 $\boldsymbol{Y}(\boldsymbol{\eta}, \dot{\boldsymbol{x}})$（式 32）:**
  $$\boldsymbol{Y}(\boldsymbol{\eta}, \dot{\boldsymbol{x}}) = -(\boldsymbol{B}\hat{\boldsymbol{I}}\boldsymbol{B}^T \boldsymbol{\omega}_{\dot{\boldsymbol{x}}})^{-1} (\dot{\boldsymbol{B}}\hat{\boldsymbol{I}}\boldsymbol{B}^T \dot{\boldsymbol{\omega}}_{\dot{\boldsymbol{x}}}  + \boldsymbol{B}\hat{\boldsymbol{I}}\boldsymbol{B}^T \boldsymbol{\omega}_{\dot{\boldsymbol{x}}})\dot{\boldsymbol{x}} \quad \text{--- (式 32)}$$ 

  前章の解説から、この $\boldsymbol{Z}(\boldsymbol{\eta})$ は、モーターから発生するトルク $\boldsymbol{F}_{\text{rot}}$ が、オイラー角の加速度 $\ddot{\boldsymbol{x}}$ にどのように影響を与えるかを表すゲイン行列です。 $\boldsymbol{Z}$ は、ドローンの姿勢（特にピッチとロールの角度）に依存して変化しますが、ヨー角には依存しません。これが、ヨー角を制御から切り離すことができる理由です。論文では静止座標から求めていますが、動座標系で計算しても同様の結果が得られます（より簡便だと思う）。

>[!NOTE]
> **制御入力ゲイン行列 $\boldsymbol{Z}(\boldsymbol{\eta})$（式 31）の求め方:**
>
> ここでは、オイラー方程式を出発点にして計算してみます（論文とは違う道筋で計算。論文の計算は後に詳説）。
>$$
>\boldsymbol{T} = \hat{\boldsymbol{I}}\dot{\boldsymbol{\Omega}} + \boldsymbol{\Omega} \times (\hat{\boldsymbol{I}}\boldsymbol{\Omega}) \quad \text{---(*2') オイラーの方程式（(式 14,15)）}
>$$
>ここで、$T = F_{rot}$ であり、$M = \boldsymbol{\Omega}_{\dot{\boldsymbol{x}}}$ とし、$\boldsymbol{\Omega} = M \dot{\boldsymbol{x}}$ と記述すると、
>$$
>\begin{align*}
>\boldsymbol{T} = \hat{\boldsymbol{I}}(M \ddot{\boldsymbol{x}} + \dot{M} \dot{\boldsymbol{x}}) + (M\dot{\boldsymbol{x}}) \times (\hat{\boldsymbol{I}} M \dot{\boldsymbol{x}})
>\end{align*}
>$$
>この式を $\ddot{\boldsymbol{x}}$ について解く。
>$$
>\begin{align*}
>\hat{\boldsymbol{I}}M \ddot{\boldsymbol{{x}}} &= \boldsymbol{T} - \hat{\boldsymbol{I}} \dot{M} \dot{\boldsymbol{x}} - (M\dot{\boldsymbol{x}}) \times (\hat{\boldsymbol{I}} M \dot{\boldsymbol{x}})\\
>\ddot{\boldsymbol{x}} &= (M^{-1} \hat{\boldsymbol{I}}^{-1}) \boldsymbol{T} - (M^{-1}\hat{\boldsymbol{I}}^{-1})(\hat{\boldsymbol{I}}\dot{M} \dot{\boldsymbol{x}} + (M\dot{\boldsymbol{x}}) \times (\hat{\boldsymbol{I}} M \dot{\boldsymbol{x}}))\\
>&= \boldsymbol{Z}(\boldsymbol{\eta}) \boldsymbol{T} + \boldsymbol{Y}(\boldsymbol{\eta}, \dot{\boldsymbol{x}})
>\end{align*}
>$$
>
>ここで、 
>$$
>\begin{align*}
>\boldsymbol{Z}(\boldsymbol{\eta}) &= M^{-1} \hat{\boldsymbol{I}}^{-1} \quad \text{--- (式 *3)}\\
>\boldsymbol{Y}(\boldsymbol{\eta}, \dot{\boldsymbol{x}}) &= - (M^{-1}\hat{\boldsymbol{I}}^{-1})(\hat{\boldsymbol{I}}\dot{M} \dot{\boldsymbol{x}} + (M\dot{\boldsymbol{x}}) \times (\hat{\boldsymbol{I}} M \dot{\boldsymbol{x}})) \quad \text{--- (式 *4)}
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
>\hat{\boldsymbol{I}}^{-1} = \begin{bmatrix}
>1/I_{11} & 0 & 0 \\
>0 & 1/I_{22} & 0 \\
>0 & 0 & 1/I_{33}
>\end{bmatrix} 
>$$
>
>また、式 *4 の外積項 $(M\dot{\boldsymbol{x}}) \times (\hat{\boldsymbol{I}}M\dot{\boldsymbol{x}})$ を、スキュー対称行列（外積と同じ働きをする交代行列）で表現することで、外積記号を消して完全に行列形式に統一できます。任意のベクトル $\boldsymbol{a}$ に対し、スキュー対称行列 $[\boldsymbol{a}]_\times$ を定義する：
>$$
>\boldsymbol{a} \times \boldsymbol{b} = [\boldsymbol{a}]_\times \boldsymbol{b}, \quad \text{ここで } \quad [\boldsymbol{a}]_\times = \begin{bmatrix} 0 & -a_3 & a_2 \\ a_3 & 0 & -a_1 \\ -a_2 & a_1 & 0 \end{bmatrix}
>$$
>
>したがって、式 *4 の外積項は、こう書ける。
>$$(M\dot{\boldsymbol{x}}) \times (\hat{\boldsymbol{I}}M\dot{\boldsymbol{x}}) = [M\dot{\boldsymbol{x}}]_\times \hat{\boldsymbol{I}}M\dot{\boldsymbol{x}} = [M\dot{\boldsymbol{x}}]_\times \hat{\boldsymbol{I}}M \dot{\boldsymbol{x}}$$
>
>**スキュー行列版**
>$$
>\boldsymbol{Y}(\boldsymbol{\eta}, \dot{\boldsymbol{x}}) = -(M^{-1}\hat{\boldsymbol{I}}^{-1})(\hat{\boldsymbol{I}}\dot{M} + [M\dot{\boldsymbol{x}}]_\times \hat{\boldsymbol{I}}M) \dot{\boldsymbol{x}} \quad \text{--- (式 *4')}
>$$


### ■ 式 *4 から式 32を導く

ここでは、式 *4 から論文の式 32 を導出する詳細なプロセスを示します。

#### ステップ2：スキュー行列による外積の統一表現

スタート **スキュー行列版**（式 *4'）

$$\boxed{\boldsymbol{Y} = -(M^{-1}\hat{\boldsymbol{I}}^{-1})(\hat{\boldsymbol{I}}\dot{M} + [M\dot{\boldsymbol{x}}]_\times \hat{\boldsymbol{I}}M)\dot{\boldsymbol{x}}}$$

#### ステップ4：座標変換による式 32 の導出

この式 *4 のスキュー行列版に対して、座標変換 $M = \boldsymbol{B}^T \boldsymbol{\omega}_{\dot{\boldsymbol{x}}}$ を適用します。

**変換関係：**
$$\begin{align*}
M &= \boldsymbol{B}^T \boldsymbol{\omega}_{\dot{\boldsymbol{x}}} \\
\dot{M} &= \dot{\boldsymbol{B}}^T \boldsymbol{\omega}_{\dot{\boldsymbol{x}}} + \boldsymbol{B}^T \dot{\boldsymbol{\omega}}_{\dot{\boldsymbol{x}}}
\end{align*}$$

**逆行列の性質：**
$$M^{-1} = \boldsymbol{\omega}_{\dot{\boldsymbol{x}}}^{-1} \boldsymbol{B}$$

**第1項の変換：**
$$-(M^{-1}\hat{\boldsymbol{I}}^{-1})(\hat{\boldsymbol{I}}\dot{M})\dot{\boldsymbol{x}} = -(\boldsymbol{\omega}_{\dot{\boldsymbol{x}}}^{-1}\boldsymbol{B}\hat{\boldsymbol{I}}^{-1}\hat{\boldsymbol{I}}\dot{\boldsymbol{B}}^T\boldsymbol{\omega}_{\dot{\boldsymbol{x}}})\dot{\boldsymbol{x}}$$

左から $\boldsymbol{\omega}_{\dot{\boldsymbol{x}}}$ を掛け、右から $\boldsymbol{\omega}_{\dot{\boldsymbol{x}}}^{-1}$ を掛けることで、静止座標系での形：

$$\dot{\boldsymbol{B}}\hat{\boldsymbol{I}}\boldsymbol{B}^T\dot{\boldsymbol{\omega}}_{\dot{\boldsymbol{x}}}$$

**第2項の変換：**
スキュー行列の座標変換性質：

$$\boldsymbol{B}[M\dot{\boldsymbol{x}}]_\times \hat{\boldsymbol{I}}M = [B M\dot{\boldsymbol{x}}]_\times \boldsymbol{B}\hat{\boldsymbol{I}}M = [\boldsymbol{\omega}]_\times \boldsymbol{B}\hat{\boldsymbol{I}}\boldsymbol{B}^T\boldsymbol{\omega}$$

ここで $\boldsymbol{\omega} = \boldsymbol{\omega}_{\dot{\boldsymbol{x}}}\dot{\boldsymbol{x}}$ です。

#### ステップ5：式 32 の確定

上記の変換をまとめると、静止座標系での形：
$$\boxed{\boldsymbol{Y}(\boldsymbol{\eta}, \dot{\boldsymbol{x}}) = -(\boldsymbol{B}\hat{\boldsymbol{I}}\boldsymbol{B}^T\boldsymbol{\omega}_{\dot{\boldsymbol{x}}})^{-1}(\dot{\boldsymbol{B}}\hat{\boldsymbol{I}}\boldsymbol{B}^T\dot{\boldsymbol{\omega}}_{\dot{\boldsymbol{x}}} + \boldsymbol{B}\hat{\boldsymbol{I}}\boldsymbol{B}^T\boldsymbol{\omega}_{\dot{\boldsymbol{x}}})\dot{\boldsymbol{x}}} \quad \text{--- (式 32)}$$

### ■ 並進の明示的な状態方程式（定理 2 / 式 33）
$$\frac{d}{dt} \begin{bmatrix} \boldsymbol{r} \\ \dot{\boldsymbol{r}} \end{bmatrix} = \begin{bmatrix} \dot{\boldsymbol{r}} \\ -g \boldsymbol{e}_3 + \frac{1}{m} \boldsymbol{B}(\boldsymbol{\phi}_1(t, (\boldsymbol{x}_0, \dot{\boldsymbol{x}}_0)^T, \boldsymbol{u})) \boldsymbol{F}_{\text{tra}}(\boldsymbol{u}) \end{bmatrix}$$
* **意味:** 重力加速度 $-g\boldsymbol{e}_3$ と、回転行列 $\boldsymbol{B}$ によって静止座標系に投影されたモータの合計推力 $\boldsymbol{F}_{\text{tra}}$ に基づく、重心位置の並進運動方程式です。

### ■ モータ配置と伝達行列（式 34〜47）
モータの回転数 $\omega_{Mi}$ （入力ゲイン $\boldsymbol{u}_4 = (\omega_{M1}^2, \omega_{M2}^2, \omega_{M3}^2, \omega_{M4}^2)^T$）からモーメントや推力を生成する変換行列を定義します。

* **クアッドコプタのモーメント伝達行列 $\boldsymbol{S}_{\text{rot}4}$（式 36）：**
  $$\boldsymbol{S}_{\text{rot}4} = \begin{bmatrix} 0 & -l k_F & 0 & l k_F \\ l k_F & 0 & -l k_F & 0 \\ -k_M & k_M & -k_M & k_M \end{bmatrix}$$
  * 1行目: ロールモーメント（左右の推力差 $\times$ アーム長 $l$）
  * 2行目: ピッチモーメント（前後の推力差 $\times$ アーム長 $l$）
  * 3行目: ヨーモーメント（時計回りと反時計回りモータの反トルク $k_M$ の差）
* **並進推力伝達行列 $\boldsymbol{S}_{\text{tra}4}$（式 45）：**
  $$\boldsymbol{S}_{\text{tra}4} = \begin{bmatrix} 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \\ k_F & k_F & k_F & k_F \end{bmatrix}$$

## 3. 4章：動作点と平衡点の定義（式 50〜55）

4章では、安定飛行または故障時における目標の飛行状態（動作点および平衡点）を定義します。
  $$\dot{\boldsymbol{x}}_{op} = \boldsymbol{0}_3$$
  $$\boldsymbol{Z}(\boldsymbol{\eta}_{op})\boldsymbol{S}_{\text{rot}2p}\boldsymbol{u}_{2p(op)} = \boldsymbol{0}_3$$
* **速度・高度（並進）に関する条件（式 51）:**
  $$\dot{\boldsymbol{r}}_{op} = \text{constant}$$
  $$\langle \boldsymbol{e}_3, -g\boldsymbol{e}_3 + \frac{1}{m}\boldsymbol{B}(\boldsymbol{x}_{op})\boldsymbol{S}_{\text{tra}2p}\boldsymbol{u}_{2p(op)} \rangle = c$$
  * $c = 0$ のときは **「定高度飛行 (Constant altitude flight)」** の状態となります。

### ■ 平衡点 (Equilibrium Point / Hovering)
定高度飛行状態（$c = 0$）において、さらに水平方向の加速度もゼロ（$\ddot{r}_1 = \ddot{r}_2 = 0$）であるならば、この動作点を**「平衡点」**と呼び、**ホバリング状態**に対応します。
* **水平加速度の式（式 53, 54）：**
  $$\ddot{r}_1 = \left\langle \boldsymbol{e}_1, \frac{1}{m}\boldsymbol{B}(\boldsymbol{x}_{op})\boldsymbol{S}_{\text{tra}2p}\boldsymbol{u}_{2p(op)} \right\rangle = 0$$
  $$\ddot{r}_2 = \left\langle \boldsymbol{e}_2, \frac{1}{m}\boldsymbol{B}(\boldsymbol{x}_{op})\boldsymbol{S}_{\text{tra}2p}\boldsymbol{u}_{2p(op)} \right\rangle = 0$$

---

## 4. 4章：モータ故障時における墜落回避制御（式 56〜57, 85〜91）

一部のモータが完全停止（$\omega_{Mi} = 0$）した状態でも、墜落を防ぐための制御可能条件を定式化します。

### ■ 正常時のフル状態（4×4）システム（定理 3 / 式 56〜57）
目標となる角加速度・鉛直加速度 $\boldsymbol{b}_4 = (\ddot{\psi}, \ddot{\theta}, \ddot{\phi}, \ddot{r}_3 + g)^T$ と、モータ入力 $\boldsymbol{u}_4$ の関係は次のように表されます。
$$\boldsymbol{A}_4(\boldsymbol{\eta})\boldsymbol{u}_4 = \boldsymbol{b}_4$$
ここで、行列 $\boldsymbol{A}_4(\boldsymbol{\eta})$ は以下のように構成されます。
$$\boldsymbol{A}_4(\boldsymbol{\eta}) = \begin{bmatrix} \boldsymbol{Z}(\boldsymbol{\eta})\boldsymbol{S}_{\text{rot}4} \\ \boldsymbol{e}_3^T \frac{1}{m}\boldsymbol{B}(\boldsymbol{x})\boldsymbol{S}_{\text{tra}4} \end{bmatrix}$$
* $\boldsymbol{A}_4(\boldsymbol{\eta})$ が正則（逆行列が存在する）であれば、一意のモータ速度 $\boldsymbol{u}_4 = \boldsymbol{A}_4^{-1}\boldsymbol{b}_4$ を求めてドローンを意のままに制御できます。

### ■ 1モータ故障時の縮小システム（式 85〜91）
クアッドコプタにおいてモータが1つ故障した場合、入力ベクトルは3次元になります。4次元の出力（ヨー、ピッチ、ロール、高度）をすべて制御することは不可能となるため、**ヨー角の制御（$\ddot{\psi}$）を放棄する（Type II 回避状態）**ことで、残りの3要素を制御します。

ヨーの行を削除した縮小システム（式 91）は次の通りです。
$$\boldsymbol{A}_{3}^1(\boldsymbol{\eta})\boldsymbol{u}_{3}^1 = \boldsymbol{b}_3$$
$$\begin{bmatrix} \boldsymbol{e}_2^T \boldsymbol{Z}(\boldsymbol{\eta})\boldsymbol{S}_{\text{rot}4}^1 \\ \boldsymbol{e}_3^T \boldsymbol{Z}(\boldsymbol{\eta})\boldsymbol{S}_{\text{rot}4}^1 \\ \boldsymbol{e}_3^T \frac{1}{m}\boldsymbol{B}(\boldsymbol{x})\boldsymbol{S}_{\text{tra}4}^1 \end{bmatrix} \begin{bmatrix} \omega_{M2}^2 \\ \omega_{M3}^2 \\ \omega_{M4}^2 \end{bmatrix} = \begin{bmatrix} \ddot{\theta} \\ \ddot{\phi} \\ \ddot{r}_3 + g \end{bmatrix}$$
* $\boldsymbol{S}_{\text{rot}4}^1$, $\boldsymbol{S}_{\text{tra}4}^1$ は、故障したモータ1に対応する1列目を削除した縮小行列です。
* この $3 \times 3$ 行列 $\boldsymbol{A}_3^1(\boldsymbol{\eta})$ が正則であれば、残存モータ速度 $\boldsymbol{u}_3^1$ を一意に決定して姿勢（ロール・ピッチ）と高度を制御し、墜落を回避することができます（ヨー軸は非制御となり、機体はスピンします）。
