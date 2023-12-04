# 2D-LiDAR-to-3D-CaveSurvey
RT components of a cave 3D surveying system using Lidar.<br>
Please check [this page](https://www.openrtm.org/openrtm/ja/node/7244).<br>
<br>
## 目的
　洞窟での測量は，地理学や地質学的な観点から重要なタスクの一つであり，センサ技術を利用したデジタル化が望まれている一分野である．特に，3 次元距離計測は洞窟内の地図構築や洞窟の形状把握が可能となるため，小型化された計測システムと解析技術の発展が望まれている．そこで，本研究では洞窟内の測量を実現するための計測機器を開発し，RT コンポーネント（RTC）により計測システムを構築することで，容易に洞窟内での計測が可能となるシステム化の有効性を示すコンポーネント群である．<br>
<br>
## 概要
・ LiDARセンサーからのデータを出力<br>
・ 3次計測を行い点群データを生成<br>
・ 複数の点群データを統合<br>
・ 統合された点群データでGrowing Neural Gas(GNG)を用いた解析<br>
・ 各種点群データの表示<br>
<br>
## 特徴
・ 各計測地点における点群データを保存<br>
・ 複数の点群データを統合<br>
・ 統合された点群データでGNGを用いた解析<br>
・ 各種点群データの表示<br>
<br>
## 仕様
・ 言語: C++<br>
・ OS: Windows 10<br>
<br>
## コンポーネント群
### 新規作成
・ EtheURG：2DLiDARセンサーを読取り出力（北陽電機 URGセンサ　UTM-30LX-EW）<br>
・ MeasurementSystem：Dynamixelを用いてLiDARを回転させ，3D点群を生成し保存・出力を行う<br>
・ Registration：複数の場所で計測された点群データを一つの点群データとして統合する<br>
・ Analyses：統合された点群データでGNGを用いた解析が行われ，その解析結果を出力する<br>
・ PointCloud_Viewer：MeasurementSystem，Registration，Analysesから受け取った点群データを表示する<br>
<br>
## ドキュメント
・ [マニュアル](https://github.com/yukimeat1999/2D-LiDAR-to-3D-CaveSurvey/blob/main/LiDAR%E3%82%92%E7%94%A8%E3%81%84%E3%81%9F%E6%B4%9E%E7%AA%9F3%E6%AC%A1%E5%85%83%E6%B8%AC%E9%87%8F%E3%82%B7%E3%82%B9%E3%83%86%E3%83%A0%E3%81%AERT%E3%82%B3%E3%83%B3%E3%83%9D%E3%83%BC%E3%83%8D%E3%83%B3%E3%83%88%E9%96%8B%E7%99%BA_v1.pdf)<br>
