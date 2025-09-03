[English](README.md) | [繁体中文](README_TCH.md) | 简体中文

# 微雪 RP2040 lcd 模拟液体

基于 waveshare RP2040-1.28-LCD 开发。

# 视频
[![IMAGE ALT TEXT](http://img.youtube.com/vi/fz67tiyyCQg/0.jpg)](https://www.youtube.com/watch?v=fz67tiyyCQg "Shake change color")

## 功能

* 模拟液体
* 格线显示
* 四方向摇动变色
* 放置一段时间休眠
* 摇动唤醒

### 和原版变更

```diff
+ 摇动唤醒
- 原版是透过 pin 来唤醒
+ 摇动变色
- 原版为单色
+ 增加说明和步骤
```

## 如何使用

### 资料和具体的环境搭建

* [微雪官方wiki RP2040-LCD-1.28]([RP2040-LCD-1.28 - Waveshare Wiki](https://www.waveshare.net/wiki/RP2040-LCD-1.28))

### 参考

* https://mitxela.com/shop/fluid-pendant
* https://mitxela.com/projects/fluid-pendant
* https://www.youtube.com/watch?v=XmzBREkK8kY
* [【流体吊坠复刻】]( https://www.bilibili.com/video/BV1PA4ZzDEAu)

### 步骤

可以自行选择build或是用build好的档案直接烧录。 (如果注重安全，应该自行build，release中应该会有提供对应版本的build。)

#### 烧录build好的档案

* 从 github 的 release 下载对应的烧录好的 `.uf2` 档案。
* 常按板子上的 boot 按钮，按住的同时接上电脑。正常而言会弹出此板子的资料夹，可以把烧录好的 `.uf2` 放到里面。 (如果没弹出资料夹可以多试几遍；或是你选择用的线或电脑端口需要更换)
* `.uf2` 档案放入后，板子会自己重启。此时就能正常使用了。

#### 自己build

* 取得一块 weshare-lcd-1.28 的板子(不要买可以触控的，脚位和程式不同，要自行修改。weshare中文是微雪。)
* 有一根能传输且稳定的线，电脑有 usb 3.2(或同等传输速度)的接口。
* 跟随微雪官方的wiki安装 arduino IDE(需要安装对应的raspberry pi模块)，以及自行安装 VScode 和里面的插件 platformIO。
* 从 github 上，clone 此专案。
* 设定好后，就能直接点 vscode 右上角的打勾按钮(platormIO 的 build)，直接 build。
* build 完后，会产生一个 release 的资料夹，里面会有 `.uf2` 的固件档案。
* 常按板子上的 boot 按钮，按住的同时接上电脑。正常而言会弹出此板子的资料夹，可以把烧录好的 `.uf2` 放到里面。 (如果没弹出资料夹可以多试几遍；或是你选择用的线或电脑端口需要更换)
* `.uf2` 档案放入后，板子会自己重启。此时就能正常使用了。

### 代码内可以修改的部分

* 液体相关的代码
  * `lib\FluidRenderer` 资料夹中。 `FluidRenderer.hpp` 里面可以改液体粒子的常数。

* 休眠模式
  * `lib\LowPower` 里面，会使用到 `LowPowerRP2040.h`，如果想要修改晃动唤醒和使用端口唤醒，可以在此更改。

* 晃动换色
  * 在液体相关的代码里面，以及 `main.cpp`里面也有部分。

###  更好的处理

* 可选
  * 外壳
    * 下载此专案内的shell档案，拿去3d打印外壳
  * 电池
    * 可以自行购买 mx1.25 端口的电池安装或自行焊接。 
  * 开关
    * 自行焊接开关，在代码内指定对应的pin来休眠或唤醒，省电量。