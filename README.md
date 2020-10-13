# TestCube_M5StackFire
IMU-powered 3D Cube

IMUで動かす3Dキューブ

<!--[Sample](Sample.jpg)-->


* Original Source</br>
M5StickC 3D COLOR CUBE</br>
https://macsbug.wordpress.com/2019/05/20/m5stickc-3d-color-cube/

 ~~It took about 12ms to depict because I removed the speeding up part from the original page above. If you use the original source page as it is, you can make it less than 10ms.~~

~~上記の元ネタページから高速化部分を削除しているので、描写に約12msかかっている。元ネタページのものをそのまま利用すれば10ms以下にできる。~~



The reason why I did that is because it behaved strangely when I just adapted the IMU pitch and roll angle to the original source material.

なぜそんなことをしたかというと、元ネタのソースにIMUのpitch,roll角を適応するだけだと、変な挙動をしたからである。

However, the real cause seems to be the point where the yaw angle is zeroed when turning a 3-dimensional angle into a 2-dimensional angle, so the yaw angle gradually drifts, which is frustrating, but I'm rotating it at a 3-dimensional angle.

しかし、真の原因は3次元の角度を2次元の角度にする際にyaw角を0するところにあるようなので、だんだんyaw角がドリフトするのが不満だが三次元の角度で回転させている。

[PS] I've incorporated the reference code by Lovyan, which allows me to portray it at less than 100Hz (10ms), which is what I've set for the Madgwick filter. Thanks.

[追記]Lovyan氏による参考コードを取り込んだことにより、Madgwickフィルタに設定している100Hz(10ms)以下で描写できるようになりました。ありがとうございます。

##  Necessary libraries

LovyanGFX</br>
https://github.com/lovyan03/LovyanGFX


## Reference

M5StickC 3D COLOR CUBE</br>
https://macsbug.wordpress.com/2019/05/20/m5stickc-3d-color-cube/

MadgwickFilter (GNU Lesser GPL)</br>
https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

Reference code by Lovyan<br/>
Lovyan氏による参考コード<br/>
https://gist.github.com/lovyan03/a4281731f6f3219aab68f665f791b148

## Translation 

DeepL</br>
https://www.deepl.com/ja/translator
