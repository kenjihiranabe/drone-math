# 論文「飛行形ドローンのモータ故障問題」数式ウォークスルー

## これは何か？

ドローンのモーターが故障した場合、他のモーターのみで安全に飛行するように制御することはできるだろうか？という問いに、この論文は、ドローンの運動を厳密にモデル化し、モーター故障時の制御可能性を定式化しています。

しかし、論文中の数式は導出プロセスがわかりにくい部分も多くあります。本ドキュメントでは、論文に登場する主要な数式について、その物理的意味や導出プロセス、座標変換の流れなどを順を追って詳しく解説します

なお、特に4プロペラのクアッドコプタを中心とした解説に寄せています。また、箱庭physicsのドローン物理エンジンの実装と対応させて解説していきます。

- 対象文献「飛行形ドローンのモータ故障問題」：岡崎秀晃、磯貝海斗
https://www.jstage.jst.go.jp/article/essfr/14/1/14_44/_pdf/-char/ja

- 参考文献「ドローンの物理シミュレータの構築」：箱庭physics
https://github.com/toppers/hakoniwa-px4sim/blob/main/drone_physics/README-ja.md

---

## 3.1節：マルチロータの運動の数学的基礎（式 1〜8）

第3.1節では、ドローンの位置と姿勢を表現するための「静止（慣性）座標系」「動（機体）座標系」の定義、および座標変換のための数式を定義します。ここでは、理論的な基礎の準備をしています。さっと流していきましょう。

### ■ アフィン空間における作用（式 1）と■ 剛体の運動の定義（式 2, 3）
$$a \mapsto a + \boldsymbol{v}, \quad a \in A^3, \; \boldsymbol{v} \in V \quad \text{--- (式 1)}$$
* **意味:** 3次元空間の幾何学的な「点」( $a$ )の集合であるアフィン空間 $A^3$ に対し、位置のズレ（相対ベクトル） $\boldsymbol{v}$ を表すベクトル空間 $V$ が平行移動として作用することを示しています。
* **物理的な位置づけ:** ドローンの移動を、点とベクトルの加算によって数学的に厳密に定義しています。
  * 静止座標系（慣性軸系）: $O + \text{span}\{\boldsymbol{e}_1, \boldsymbol{e}_2, \boldsymbol{e}_3\}$
  * 動座標系（質量中心 $O_c$ に固定）: $O_c + \text{span}\{\boldsymbol{E}_1, \boldsymbol{E}_2, \boldsymbol{E}_3\}$

この座標系において、剛体運動を定義します。並行移動と回転移動の組み合わせです。

$$\boldsymbol{T}(t): W \to w \quad \text{--- (式 2)}$$
$$\boldsymbol{T}(t) = \boldsymbol{C}(t)\boldsymbol{B}(t) \quad \text{--- (式 3)}$$

式(2)は、剛体（ドローン機体）の運動 $\boldsymbol{T}(t)$ は、動座標系 $W$ から静止座標系 $w$ へのアフィン写像として定義されます（写像の「定義域＝入力の集合 $W$ 」と「終域＝出力の集合 $w$ 」）。変換の向きに注意してください。動座標系のベクトルを静止座標系へと変換します。本論文では、一貫して動座標系→静止座標系が順方向です。この方が変換行列の表記として自然になります。

式(3)は、この運動が **「回転行列 $\boldsymbol{B}(t)$」** と **「並進（位置の移動）$\boldsymbol{C}(t)$」** の合成として一意に表せることを示しています。ここで、 $\boldsymbol{B}(t)\boldsymbol{C}(t)$ は、あたかも掛け算のように書かれていますが、
実際には掛け算ではなく、「回転行列 $\boldsymbol{B}(t)$ による座標変換」と「並進ベクトル $\boldsymbol{C}(t)$ による位置の移動」の
**合成変換**（すなわちアフィン変換）を表しています。また、両者が時間 $t$ の関数になっていることにも注意してください。

### ■ 動径ベクトルと速度ベクトル（式 4, 5）

これを具体的な位置ベクトルと速度ベクトルの式に展開していきます。なお、一貫して小文字で静止座標系のベクトル（ $w$ の元）、大文字で動座標系のベクトル（ $W$ の元）を表す慣習が続きます。


$$\boldsymbol{q}(t) = \boldsymbol{r}(t) + \boldsymbol{B}(t)\boldsymbol{Q}(t) \quad \text{--- (式 4)}$$
$$\dot{\boldsymbol{q}} = \dot{\boldsymbol{r}} + \dot{\boldsymbol{B}}\boldsymbol{Q} + \boldsymbol{B}\dot{\boldsymbol{Q}} \quad \text{--- (式 5)}$$

式(4, 5)で、$\boldsymbol{q}, \boldsymbol{r} \in w$ は、静止座標系における位置ベクトルであり、$\boldsymbol{Q} \in W$ は、動座標系における位置ベクトル（動径ベクトルと呼んでいる）を表しています。

* **式(4)の意味:** 静止座標系から見た動点の位置 $\boldsymbol{q}(t) \in w$ は、「重心の位置 $\boldsymbol{r}(t) \in w$」に「機体内の相対位置 $\boldsymbol{Q}(t) \in W$ を回転行列 $\boldsymbol{B}(t)$ で静止座標系へ変換したベクトル $\boldsymbol{B}(t)\boldsymbol{Q}(t) \in w$ 」を加えたものです。
* **式(5)の意味:** これを時間微分した速度式です。機体が変形しない剛体であれば、機体内での相対位置の変化はないため、第3項 $\boldsymbol{B}\dot{\boldsymbol{Q}} = \boldsymbol{0}$ となります。

この(式4,5)から、非常に汎用的な運動方程式(式13)が後に導出されます。さらに準備として、ドローンの姿勢を表すテイト-ブライアン角（ヨー、ピッチ、ロール）に対応する基本回転行列を定義します。

### ■ テイト-ブライアン角に対応する基本回転行列（式 6, 7, 8）

ドローンの姿勢（回転）は、ヨー（$\psi$）、ピッチ（$\theta$）、ロール（$\phi$）の3つの軸まわりの回転を、この順に静止座標系の基底を回転して動座標系の基底に重ねることで表現します（テイト・ブライアン角）。3つの角の順番には様々な流儀（12種類）があるので注意してください。これらを横に並べて表現すると以下の通りです：

| ヨー回転 $\boldsymbol{R}_{\psi}$<br>（Z軸まわり、式 6） | ピッチ回転 $\boldsymbol{R}_{\theta}$<br>（中間Y軸まわり、式 7） | ロール回転 $\boldsymbol{R}_{\phi}$<br>（機体X軸まわり、式 8） |
| :---: | :---: | :---: |
| $\boldsymbol{R}_{\psi} = \begin{pmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{pmatrix}$ | $\boldsymbol{R}_{\theta} = \begin{pmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{pmatrix}$ | $\boldsymbol{R}_{\phi} = \begin{pmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{pmatrix}$ |

注：これは箱庭physicsで採用されているオイラー角と同じ順番、同じ記号の右手形です。ただし箱庭はz軸が下向きである点が異なります。
箱庭physics(https://github.com/toppers/hakoniwa-px4sim/blob/main/drone_physics/README-ja.md)では、
- 機体座標系 body frame は FRD(front-right-down)。
- 地上座標系 ground frame は NED(north-east-down)。

理論的には重力方向の扱いと制御時のラダー・エレベータ・エルロン・ラダーの符号が異なりますが、以下の論文と基本的には箱庭physicsとそのまま同じ式が成り立ちます。

また、ドローンの姿勢を表す回転行列 $\boldsymbol{B}$ は、これらの基本回転行列を掛け合わせることで構成されます（式 9）。この合成回転行列 $\boldsymbol{B}$ を用いて、機体固定座標系から静止座標系への変換を行います。

$$\boldsymbol{B}(\psi, \theta, \phi) = \boldsymbol{R}_{\psi}\boldsymbol{R}_{\theta}\boldsymbol{R}_{\phi}$$

> [!TIP]
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
>- $\boldsymbol{B}$ はある１つの軸周りの回転として定義できる。その軸は $\boldsymbol{B}$ の固有値1に対応する固有ベクトルである。

| 静止座標系 $\{\boldsymbol{e}_1, \boldsymbol{e}_2, \boldsymbol{e}_3\}$ での座標 | 変換 | 動座標系 $\{\boldsymbol{E}_1, \boldsymbol{E}_2, \boldsymbol{E}_3\}$ での座標 |
| :---: | :---: | :---: |
| <br>速度$\boldsymbol{v}$<br>角速度$\boldsymbol{\omega}$ |  $\boldsymbol{B}^T$を掛ける<br>$\rightarrow$<br>$\leftarrow$<br>$\boldsymbol{B}$ を掛ける   | <br>速度 $\boldsymbol{V}$<br> 角速度 $\boldsymbol{V}$ |


### ■ 回転座標系における速度と加速度の展開（式 10, 13）

座標系が回転している環境下での、位置ベクトルの微分（速度・加速度）の展開式です。本論文では、ベクトル積（外積）をブラケット記法 $[\boldsymbol{a}, \boldsymbol{b}] = \boldsymbol{a} \times \boldsymbol{b}$ で表現しています。

* **速度ベクトル（式 10）：**
  機体が角速度 $\boldsymbol{\Omega}$ (動座標系) または $\boldsymbol{\omega}$ (静止座標系) で回転しているとき、動径ベクトル $\boldsymbol{Q}$ の微分は次のようになります。
  $$\dot{\boldsymbol{q}} = \dot{\boldsymbol{r}} + \dot{\boldsymbol{B}}\boldsymbol{Q} + \boldsymbol{B}\dot{\boldsymbol{Q}} = \dot{\boldsymbol{r}} + \boldsymbol{B} [\boldsymbol{\Omega}, \boldsymbol{Q}] + \boldsymbol{B}\dot{\boldsymbol{Q}}$$

機体内固定の点であれば $\dot{\boldsymbol{Q}} = \boldsymbol{0}$ となり、論文中の
$$
\dot{\boldsymbol{q}} = \dot{\boldsymbol{r}} + [\boldsymbol{\omega}, \boldsymbol{B}\boldsymbol{Q}] = \dot{\boldsymbol{r}} + \boldsymbol{B}[\boldsymbol{\Omega}, \boldsymbol{Q}] = \dot{\boldsymbol{r}} + [\boldsymbol{B}\boldsymbol{\Omega}, \boldsymbol{B}\boldsymbol{Q}]
$$
 と等価になります。
  また、 $\boldsymbol{B} [\boldsymbol{\Omega}, \boldsymbol{Q}] = [\boldsymbol{B}\boldsymbol{\Omega}, \boldsymbol{B}\boldsymbol{Q}]$ となる理由は、回転行列 $\boldsymbol{B}$ は外積の構造を保つためです。すなわち、2つのベクトルの外積をとってから回転しても、2つのベクトルをあらかじめ回転させてからその外積をとっても同じ結果になります。
  
* **加速度ベクトル（式 13）：**
  速度ベクトル式をさらにもう一度時間微分することで、回転系における加速度方程式を得ます。
  $$\ddot{\boldsymbol{q}} = \ddot{\boldsymbol{r}} + \boldsymbol{B}\ddot{\boldsymbol{Q}} + 2\dot{\boldsymbol{B}}\dot{\boldsymbol{Q}} + \ddot{\boldsymbol{B}}\boldsymbol{Q}$$
  $$\ddot{\boldsymbol{q}} = \ddot{\boldsymbol{r}} + \boldsymbol{B} [\boldsymbol{\Omega}, [\boldsymbol{\Omega}, \boldsymbol{Q}]] + \boldsymbol{B} [\dot{\boldsymbol{\Omega}}, \boldsymbol{Q}] + 2\boldsymbol{B} [\boldsymbol{\Omega}, \dot{\boldsymbol{Q}}] + \boldsymbol{B}\ddot{\boldsymbol{Q}} \quad \text{--- (動座標系)}$$
  $$\ddot{\boldsymbol{q}} = \ddot{\boldsymbol{r}} + [\boldsymbol{\omega}, [\boldsymbol{\omega}, \boldsymbol{B}\boldsymbol{Q}]] + [\dot{\boldsymbol{\omega}}, \boldsymbol{B}\boldsymbol{Q}] + 2[\boldsymbol{\omega}, \boldsymbol{B}\dot{\boldsymbol{Q}}] + \boldsymbol{B}\ddot{\boldsymbol{Q}} \quad \text{--- (静止座標系)}$$

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

本論文では、剛体の運動のみを扱う（ $\dot{\boldsymbol{Q}} = \boldsymbol{0}, \ddot{\boldsymbol{Q}} = \boldsymbol{0}$ ）ため、4.のColioris力の項、5.の相対加速度の項は消えます。
さらに、$\boldsymbol{r}$ をドローンの重心位置としているため（ $\Sigma m_i \boldsymbol{Q}_i = 0$ ）、 $\boldsymbol{Q}$ がそのまま関与する 2.の遠心力、3.のオイラー加速度の項も釣り合って消えます。すると、実際に本論文で使われる重心の並進に関する式は、

$$\ddot{\boldsymbol{q}} = \ddot{\boldsymbol{r}} \quad \text{--- (式 13)’}$$

となるだけです。

ただし、ここから、動座標系での並進の運動方程式を立式する際には、やはり見かけの力が（再度）発生します。
重心の速度 $\boldsymbol{v} = \dot{\boldsymbol{r}}$ とし、

$$
\boldsymbol{f} = m\dot{\boldsymbol{v}} \quad \text{--- (*1)}
$$

という重心に対するニュートンの方程式を立式します（ $\boldsymbol{f} \in w$ ）。
重心の速度 $\boldsymbol{v} = \dot{\boldsymbol{r}} \in w$ を動座標系で表して $\boldsymbol{V} \in W$ とすると $\boldsymbol{v} = \boldsymbol{B}\boldsymbol{V}$ となるため、輸送定理（すぐ後に解説）により、式13'は、

$$\ddot{\boldsymbol{r}} = \dot{\boldsymbol{v}} = \boldsymbol{B} (\dot{\boldsymbol{V}} + \boldsymbol{\Omega} \times \boldsymbol{V})$$

となり、重心についても慣性力がかかります。(*)は、

$$
\boldsymbol{f} = m \dot{\boldsymbol{v}} = m \boldsymbol{B} (\dot{\boldsymbol{V}} + \boldsymbol{\Omega} \times \boldsymbol{V})
$$
動座標系で表すと、 $\boldsymbol{F} = \boldsymbol{B}^T \boldsymbol{f} \in W$ として、
$$
\boldsymbol{F} = m\dot{\boldsymbol{V}} + m \boldsymbol{\Omega} \times \boldsymbol{V}
$$

となり、コリオリ力に相当する項が再度現れます。原点（重心）の並進に対してかかるコリオリ力です。
一旦、剛体条件で消えたのですが、並進の中にも含まれていたのですね。そして、この式が、ドローンの運動方程式の基礎となり、後の状態方程式の導出に繋がっていきます。


#### 💡 式 13 の数学的導出メモ
回転行列 $\boldsymbol{B}(t)$、その回転角速度ベクトルを $\boldsymbol{\Omega}$とすると、任意の動座標ベクトル  $\boldsymbol{Q}$ に対して、

**ポアソンの関係式** ：
$$\dot{\boldsymbol{B}}\boldsymbol{Q} = \boldsymbol{B} (\boldsymbol{\Omega} \times \boldsymbol{Q})$$
および、対応する静止座標ベクトル $\boldsymbol{q} = \boldsymbol{B}\boldsymbol{Q}$ とすると、

**輸送定理** ：
$$\frac{d}{dt} \boldsymbol{q} = \boldsymbol{B} (\boldsymbol{\Omega} \times \boldsymbol{Q} + \frac{d}{dt}\boldsymbol{Q})$$

すなわち、「静止座標での時間微分を動座標の時間微分に変換する際、 $\boldsymbol{\Omega}$ の外積を掛けたものをプラスする」。

これらを使って、
$\ddot{\boldsymbol{B}}\boldsymbol{Q}$ を求めます。
積の微分公式を用いると：
$$\frac{d}{dt} (\dot{\boldsymbol{B}}\boldsymbol{Q}) = \ddot{\boldsymbol{B}}\boldsymbol{Q} + \dot{\boldsymbol{B}}\dot{\boldsymbol{Q}} = \frac{d}{dt} \left( \boldsymbol{B} (\boldsymbol{\Omega} \times \boldsymbol{Q}) \right) = \dot{\boldsymbol{B}} (\boldsymbol{\Omega} \times \boldsymbol{Q}) + \boldsymbol{B} (\dot{\boldsymbol{\Omega}} \times \boldsymbol{Q}) + \boldsymbol{B} (\boldsymbol{\Omega} \times \dot{\boldsymbol{Q}})$$
ここで、第1項に再びポアソンの公式 $\dot{\boldsymbol{B}}\boldsymbol{v} = \boldsymbol{B}(\boldsymbol{\Omega} \times \boldsymbol{v})$ を適用（$\boldsymbol{v} = \boldsymbol{\Omega} \times \boldsymbol{Q}$）します。
$$\dot{\boldsymbol{B}} (\boldsymbol{\Omega} \times \boldsymbol{Q}) = \boldsymbol{B} (\boldsymbol{\Omega} \times (\boldsymbol{\Omega} \times \boldsymbol{Q}))$$
これを元の式に代入すると：
$$\ddot{\boldsymbol{B}}\boldsymbol{Q} + \dot{\boldsymbol{B}}\dot{\boldsymbol{Q}} = \boldsymbol{B} (\boldsymbol{\Omega} \times (\boldsymbol{\Omega} \times \boldsymbol{Q})) + \boldsymbol{B} (\dot{\boldsymbol{\Omega}} \times \boldsymbol{Q}) + \boldsymbol{B} (\boldsymbol{\Omega} \times \dot{\boldsymbol{Q}})$$
両辺から $\dot{\boldsymbol{B}}\dot{\boldsymbol{Q}} = \boldsymbol{B}(\boldsymbol{\Omega} \times \dot{\boldsymbol{Q}})$ を減算すれば、$\ddot{\boldsymbol{B}}\boldsymbol{Q}$ の展開式が得られます。
$$\ddot{\boldsymbol{B}}\boldsymbol{Q} = \boldsymbol{B} (\boldsymbol{\Omega} \times (\boldsymbol{\Omega} \times \boldsymbol{Q})) + \boldsymbol{B} (\dot{\boldsymbol{\Omega}} \times \boldsymbol{Q})$$
この $\ddot{\boldsymbol{B}}\boldsymbol{Q}$ を、$\ddot{\boldsymbol{q}} = \ddot{\boldsymbol{r}} + \boldsymbol{B}\ddot{\boldsymbol{Q}} + 2\dot{\boldsymbol{B}}\dot{\boldsymbol{Q}} + \ddot{\boldsymbol{B}}\boldsymbol{Q}$ に代入すると、キャンセルされる項などを経て、式 13 の第2行目が導き出されます。さらに、$\boldsymbol{B}(\boldsymbol{\Omega} \times \boldsymbol{v}) = \boldsymbol{\omega} \times (\boldsymbol{B}\boldsymbol{v})$ という回転投影の関係を用いれば、静止座標系での表現（第3行目）へと変換されます。

---

## 2. 3.2節：マルチロータの状態方程式（式 9〜49）

第3.2節では、ドローンの姿勢変化と力の伝達、およびそれらを統一した非線形の状態方程式（運動方程式）を組み立てます。

### ■ 合成回転行列 $\boldsymbol{B}$ （式 9）
$$\boldsymbol{B}(\psi, \theta, \phi) = \boldsymbol{R}_{\psi}\boldsymbol{R}_{\theta}\boldsymbol{R}_{\phi} = \begin{pmatrix} \cos\psi\cos\theta & \cos\psi\sin\theta\sin\phi - \sin\psi\cos\phi & \cos\psi\sin\theta\cos\phi + \sin\psi\sin\phi \\ \sin\psi\cos\theta & \sin\psi\sin\theta\sin\phi + \cos\psi\cos\phi & \sin\psi\sin\theta\cos\phi - \cos\psi\sin\phi \\ -\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi \end{pmatrix}$$
* **意味:** 3つの基本回転を組み合わせた、動座標系ベクトルから静止座標系ベクトルへの変換行列です。

### ■ 角速度ベクトルとオイラー角の微分（式 11, 12）
$$\boldsymbol{\Omega} = \boldsymbol{B}^T \boldsymbol{\omega} \quad \text{--- (式 11)}$$
$$\boldsymbol{\omega} = \dot{\psi}\boldsymbol{e}_3 + \dot{\theta}\boldsymbol{E}_2^{(-2)} + \dot{\phi}\boldsymbol{E}_1^{(-1)} \quad \text{--- (式 12)}$$
* **意味:** 機体固定の瞬間角速度 $\boldsymbol{\Omega}$ と空間角速度 $\boldsymbol{\omega}$ の関係式、およびオイラー角の微分（$\dot{\psi}, \dot{\theta}, \dot{\phi}$）が空間角速度 $\boldsymbol{\omega}$ とどう結びついているかを示します。

(式12)は、この後でてこないので、各オイラー角の時間微分が変換途中の座標軸に対してどのように空間角速度で構成されることに注意するだけでよいです。
(式11)はこの後何度も出てきます。逆方向の変換と合わせて覚えておきます。動系から静止系へのベクトルの変換は、 $\boldsymbol{B}$ を掛けることで行われることを思い出しまししょう。各速度はベクトルですので、この規則が当てはまります（オイラー角には当てはまりません）。

$$\boldsymbol{\Omega} = \boldsymbol{B}^T \boldsymbol{\omega} \quad \text{--- (式 11)}$$
$$\boldsymbol{\omega} = \boldsymbol{B} \boldsymbol{\Omega} \quad \text{--- (式 11)'}$$

を構成するかを示す式です。これを式11の関係式と組み合わせることで、オイラー角の微分から空間角速度への変換行列 $\boldsymbol{\omega}_{\dot{\boldsymbol{x}}}(\psi, \theta)$ を導出することができます。

### ■ オイラーの運動方程式（式 14, 15）
$$\boldsymbol{h} = \boldsymbol{I}(\boldsymbol{x})\boldsymbol{\omega} \quad \text{--- (式 14)}$$
$$\dot{\boldsymbol{h}} = \boldsymbol{\tau} \quad \text{--- (式 15)}$$
* **意味:** 静止座標系における角運動量 $\boldsymbol{h}$ とトルク $\boldsymbol{\tau}$ の関係です。

### ■ システムのラグランジアン $L$ と一般化力（式 22, 23）
$$L = \frac{1}{2} m \langle \dot{\boldsymbol{r}}, \dot{\boldsymbol{r}} \rangle + \frac{1}{2} \langle \hat{\boldsymbol{I}}\boldsymbol{\Omega}, \boldsymbol{\Omega} \rangle - mg \langle \boldsymbol{r}, \boldsymbol{e}_3 \rangle \quad \text{--- (式 22)}$$
$$\langle \boldsymbol{B}(\boldsymbol{x})\boldsymbol{F}_{\text{rot}}(\boldsymbol{u}), \delta\boldsymbol{\omega} \rangle = \langle \boldsymbol{\omega}_{\dot{\boldsymbol{x}}}(\psi, \theta)^T \boldsymbol{B}(\boldsymbol{x})\boldsymbol{F}_{\text{rot}}(\boldsymbol{u}), \delta\dot{\boldsymbol{x}} \rangle \quad \text{--- (式 23)}$$
* **第22式:** 並進の運動エネルギー、回転の運動エネルギー、重力ポテンシャルから構成されるラグランジアンです。
* **第23式:** 仮想仕事の原理に基づき、モータによるモーメント $\boldsymbol{F}_{\text{rot}}$ をオイラー角 $\boldsymbol{x}$ の一般化力へ変換します。

* **変換行列 $\boldsymbol{\omega}_{\dot{\boldsymbol{x}}}(\psi, \theta)$（式 28）：**
  $$\boldsymbol{\omega}_{\dot{\boldsymbol{x}}}(\psi, \theta) = \begin{pmatrix} 0 & -\sin\psi & \cos\theta \cos\psi \\ 0 & \cos\psi & \cos\theta \sin\psi \\ 1 & 0 & -\sin\theta \end{pmatrix}$$

### ■ 回転の明示的な状態方程式（定理 1 / 式 29〜32）
$$\frac{d}{dt} \begin{pmatrix} \boldsymbol{x} \\ \dot{\boldsymbol{x}} \end{pmatrix} = \begin{pmatrix} \dot{\boldsymbol{x}} \\ \boldsymbol{Y}(\boldsymbol{\eta}, \dot{\boldsymbol{x}}) + \boldsymbol{Z}(\boldsymbol{\eta})\boldsymbol{F}_{\text{rot}}(\boldsymbol{u}) \end{pmatrix} \quad \text{--- (式 30)}$$

* **制御入力ゲイン行列 $\boldsymbol{Z}(\boldsymbol{\eta})$（式 31）:**
  $$\boldsymbol{Z}(\boldsymbol{\eta}) = (\boldsymbol{B}(\boldsymbol{x})\hat{\boldsymbol{I}}\boldsymbol{B}(\boldsymbol{x})^T \cdot \boldsymbol{\omega}_{\dot{\boldsymbol{x}}}(\psi, \theta))^{-1} \cdot \boldsymbol{B}(\boldsymbol{x})$$
* **非線形項（コリオリ・遠心力など） $\boldsymbol{Y}(\boldsymbol{\eta}, \dot{\boldsymbol{x}})$（式 32）:**
  $$\boldsymbol{Y}(\boldsymbol{\eta}, \dot{\boldsymbol{x}}) = -(\boldsymbol{B}(\boldsymbol{x})\hat{\boldsymbol{I}}\boldsymbol{B}(\boldsymbol{x})^T\boldsymbol{\omega}_{\dot{\boldsymbol{x}}}(\psi, \theta))^{-1} \left[ \dot{\boldsymbol{B}}(\boldsymbol{x}, \dot{\boldsymbol{x}})\hat{\boldsymbol{I}}\boldsymbol{B}(\boldsymbol{x})^T \cdot \boldsymbol{\omega}_{\dot{\boldsymbol{x}}}(\psi, \theta) + \boldsymbol{B}(\boldsymbol{x})\hat{\boldsymbol{I}}\boldsymbol{B}(\boldsymbol{x})^T \cdot \dot{\boldsymbol{\omega}}_{\dot{\boldsymbol{x}}}(\psi, \theta, \dot{\psi}, \dot{\theta}) \right] \dot{\boldsymbol{x}}$$

> [!TIP]
> **【詳細導出】式 32 は以下のようにニュートン・オイラー方程式から導かれます：**
> 1. 静止座標系における角運動量方程式 $\dot{\boldsymbol{h}} = \boldsymbol{\tau}$ から出発します。ここで、静止座標系から見た慣性モーメントを $\boldsymbol{I}(\boldsymbol{x})$、機体座標系での慣性モーメントを $\hat{\boldsymbol{I}}$ とおくと、その関係は $\boldsymbol{I}(\boldsymbol{x}) = \boldsymbol{B}(\boldsymbol{x})\hat{\boldsymbol{I}}\boldsymbol{B}(\boldsymbol{x})^T$ です。静止座標系における角運動量は $\boldsymbol{h} = \boldsymbol{I}(\boldsymbol{x})\boldsymbol{\omega}$、トルクは $\boldsymbol{\tau} = \boldsymbol{B}(\boldsymbol{x})\boldsymbol{F}_{\text{rot}}(\boldsymbol{u})$ と表されます。
> 2. 時間微分を展開すると $\dot{\boldsymbol{h}} = \boldsymbol{I}(\boldsymbol{x})\dot{\boldsymbol{\omega}} + \dot{\boldsymbol{I}}(\boldsymbol{x}, \dot{\boldsymbol{x}})\boldsymbol{\omega}$ となります。ここで、ポアソンの公式について、機体座標系の角速度 $\boldsymbol{\Omega}$ を用いると $\dot{\boldsymbol{B}} = \boldsymbol{B}[\boldsymbol{\Omega}]_{\times}$ ですが、空間（慣性）座標系の角速度 $\boldsymbol{\omega} = \boldsymbol{B}\boldsymbol{\Omega}$ を用いると、外積行列の回転変換特性 $\boldsymbol{B}[\boldsymbol{v}]_{\times}\boldsymbol{B}^T = [\boldsymbol{B}\boldsymbol{v}]_{\times}$ より、左から掛ける形 $\dot{\boldsymbol{B}} = [\boldsymbol{\omega}]_{\times}\boldsymbol{B}$ となります（※ $\dot{\boldsymbol{B}} = \boldsymbol{B}[\boldsymbol{B}^T\boldsymbol{\omega}]_{\times} = [\boldsymbol{B}\boldsymbol{B}^T\boldsymbol{\omega}]_{\times}\boldsymbol{B} = [\boldsymbol{\omega}]_{\times}\boldsymbol{B}$ ）。これと $[\boldsymbol{\omega}]_{\times}\boldsymbol{\omega} = \boldsymbol{0}$ より、第2項は以下のように変形できます。
>    $$\dot{\boldsymbol{I}}\boldsymbol{\omega} = (\dot{\boldsymbol{B}}\hat{\boldsymbol{I}}\boldsymbol{B}^T + \boldsymbol{B}\hat{\boldsymbol{I}}\dot{\boldsymbol{B}}^T)\boldsymbol{\omega} = \dot{\boldsymbol{B}}\hat{\boldsymbol{I}}\boldsymbol{B}^T\boldsymbol{\omega}$$
>    これにより運動方程式は $\boldsymbol{I}(\boldsymbol{x})\dot{\boldsymbol{\omega}} + \dot{\boldsymbol{B}}(\boldsymbol{x}, \dot{\boldsymbol{x}})\hat{\boldsymbol{I}}\boldsymbol{B}(\boldsymbol{x})^T\boldsymbol{\omega} = \boldsymbol{B}(\boldsymbol{x})\boldsymbol{F}_{\text{rot}}(\boldsymbol{u})$ と簡略化されます。
> 3. オイラー角の微分と空間角速度の関係 $\boldsymbol{\omega} = \boldsymbol{\omega}_{\dot{\boldsymbol{x}}}\dot{\boldsymbol{x}}$ を微分して空間角加速度 $\dot{\boldsymbol{\omega}} = \boldsymbol{\omega}_{\dot{\boldsymbol{x}}}\ddot{\boldsymbol{x}} + \dot{\boldsymbol{\omega}}_{\dot{\boldsymbol{x}}}\dot{\boldsymbol{x}}$ を得ます。
> 4. これらを運動方程式に代入し、$\boldsymbol{I}(\boldsymbol{x}) = \boldsymbol{B}(\boldsymbol{x})\hat{\boldsymbol{I}}\boldsymbol{B}(\boldsymbol{x})^T$ を用いて整理すると：
>    $$\boldsymbol{B}\hat{\boldsymbol{I}}\boldsymbol{B}^T\boldsymbol{\omega}_{\dot{\boldsymbol{x}}}\ddot{\boldsymbol{x}} + \left( \dot{\boldsymbol{B}}\hat{\boldsymbol{I}}\boldsymbol{B}^T \boldsymbol{\omega}_{\dot{\boldsymbol{x}}} + \boldsymbol{B}\hat{\boldsymbol{I}}\boldsymbol{B}^T\dot{\boldsymbol{\omega}}_{\dot{\boldsymbol{x}}} \right) \dot{\boldsymbol{x}} = \boldsymbol{B}\boldsymbol{F}_{\text{rot}}$$
>    左から逆行列 $(\boldsymbol{B}\hat{\boldsymbol{I}}\boldsymbol{B}^T\boldsymbol{\omega}_{\dot{\boldsymbol{x}}})^{-1}$ を掛けることで、式 32 の形で非線形項 $\boldsymbol{Y}(\boldsymbol{\eta}, \dot{\boldsymbol{x}})$ が導かれます。

### ■ 並進の明示的な状態方程式（定理 2 / 式 33）
$$\frac{d}{dt} \begin{pmatrix} \boldsymbol{r} \\ \dot{\boldsymbol{r}} \end{pmatrix} = \begin{pmatrix} \dot{\boldsymbol{r}} \\ -g \boldsymbol{e}_3 + \frac{1}{m} \boldsymbol{B}(\boldsymbol{\phi}_1(t, (\boldsymbol{x}_0, \dot{\boldsymbol{x}}_0)^T, \boldsymbol{u})) \boldsymbol{F}_{\text{tra}}(\boldsymbol{u}) \end{pmatrix}$$
* **意味:** 重力加速度 $-g\boldsymbol{e}_3$ と、回転行列 $\boldsymbol{B}$ によって静止座標系に投影されたモータの合計推力 $\boldsymbol{F}_{\text{tra}}$ に基づく、重心位置の並進運動方程式です。

### ■ モータ配置と伝達行列（式 34〜47）
モータの回転数 $\omega_{Mi}$ （入力ゲイン $\boldsymbol{u}_4 = (\omega_{M1}^2, \omega_{M2}^2, \omega_{M3}^2, \omega_{M4}^2)^T$）からモーメントや推力を生成する変換行列を定義します。

* **クアッドコプタのモーメント伝達行列 $\boldsymbol{S}_{\text{rot}4}$（式 36）：**
  $$\boldsymbol{S}_{\text{rot}4} = \begin{pmatrix} 0 & -l k_F & 0 & l k_F \\ l k_F & 0 & -l k_F & 0 \\ -k_M & k_M & -k_M & k_M \end{pmatrix}$$
  * 1行目: ロールモーメント（左右の推力差 $\times$ アーム長 $l$）
  * 2行目: ピッチモーメント（前後の推力差 $\times$ アーム長 $l$）
  * 3行目: ヨーモーメント（時計回りと反時計回りモータの反トルク $k_M$ の差）
* **並進推力伝達行列 $\boldsymbol{S}_{\text{tra}4}$（式 45）：**
  $$\boldsymbol{S}_{\text{tra}4} = \begin{pmatrix} 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \\ k_F & k_F & k_F & k_F \end{pmatrix}$$

---

## 3. 4章：動作点と平衡点の定義（式 50〜55）

4章では、安定飛行または故障時における目標の飛行状態（動作点および平衡点）を定義します。

### ■ 動作点 (Operating Point) の定義（式 50, 51）
姿勢変化がなく（$\dot{\boldsymbol{x}}_{op} = \boldsymbol{0}_3$）、並進が一定速度（一定加速度）で動いている状態です。

* **姿勢（回転）に関する条件（式 50）:**
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
$$\boldsymbol{A}_4(\boldsymbol{\eta}) = \begin{pmatrix} \boldsymbol{Z}(\boldsymbol{\eta})\boldsymbol{S}_{\text{rot}4} \\ \boldsymbol{e}_3^T \frac{1}{m}\boldsymbol{B}(\boldsymbol{x})\boldsymbol{S}_{\text{tra}4} \end{pmatrix}$$
* $\boldsymbol{A}_4(\boldsymbol{\eta})$ が正則（逆行列が存在する）であれば、一意のモータ速度 $\boldsymbol{u}_4 = \boldsymbol{A}_4^{-1}\boldsymbol{b}_4$ を求めてドローンを意のままに制御できます。

### ■ 1モータ故障時の縮小システム（式 85〜91）
クアッドコプタにおいてモータが1つ故障した場合、入力ベクトルは3次元になります。4次元の出力（ヨー、ピッチ、ロール、高度）をすべて制御することは不可能となるため、**ヨー角の制御（$\ddot{\psi}$）を放棄する（Type II 回避状態）**ことで、残りの3要素を制御します。

ヨーの行を削除した縮小システム（式 91）は次の通りです。
$$\boldsymbol{A}_{3}^1(\boldsymbol{\eta})\boldsymbol{u}_{3}^1 = \boldsymbol{b}_3$$
$$\begin{pmatrix} \boldsymbol{e}_2^T \boldsymbol{Z}(\boldsymbol{\eta})\boldsymbol{S}_{\text{rot}4}^1 \\ \boldsymbol{e}_3^T \boldsymbol{Z}(\boldsymbol{\eta})\boldsymbol{S}_{\text{rot}4}^1 \\ \boldsymbol{e}_3^T \frac{1}{m}\boldsymbol{B}(\boldsymbol{x})\boldsymbol{S}_{\text{tra}4}^1 \end{pmatrix} \begin{pmatrix} \omega_{M2}^2 \\ \omega_{M3}^2 \\ \omega_{M4}^2 \end{pmatrix} = \begin{pmatrix} \ddot{\theta} \\ \ddot{\phi} \\ \ddot{r}_3 + g \end{pmatrix}$$
* $\boldsymbol{S}_{\text{rot}4}^1$, $\boldsymbol{S}_{\text{tra}4}^1$ は、故障したモータ1に対応する1列目を削除した縮小行列です。
* この $3 \times 3$ 行列 $\boldsymbol{A}_3^1(\boldsymbol{\eta})$ が正則であれば、残存モータ速度 $\boldsymbol{u}_3^1$ を一意に決定して姿勢（ロール・ピッチ）と高度を制御し、墜落を回避することができます（ヨー軸は非制御となり、機体はスピンします）。
