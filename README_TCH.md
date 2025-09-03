[English](README.md) | 繁體中文 | [簡體中文](README_SCH.md)

# 微雪 RP2040 lcd 模擬液體 

 基於 waveshare RP2040-1.28-LCD 開發。

# 影片
[![IMAGE ALT TEXT](http://img.youtube.com/vi/fz67tiyyCQg/0.jpg)](https://www.youtube.com/watch?v=fz67tiyyCQg "Shake change color")

## 功能

* 模擬液體
* 格線顯示
* 四方向搖動變色
* 放置一段時間休眠
* 搖動喚醒

### 和原版變更
```diff
+ 搖動喚醒
- 原版是透過 pin 來喚醒
+ 搖動變色
- 原版為單色
+ 增加說明和步驟
```

## 如何使用

### 資料和具體的環境搭建

* [微雪官方wiki RP2040-LCD-1.28]([RP2040-LCD-1.28 - Waveshare Wiki](https://www.waveshare.net/wiki/RP2040-LCD-1.28))

### 參考
* https://mitxela.com/shop/fluid-pendant
* https://mitxela.com/projects/fluid-pendant
* https://www.youtube.com/watch?v=XmzBREkK8kY
* [【流体吊坠复刻】]( https://www.bilibili.com/video/BV1PA4ZzDEAu)

### 步驟

可以自行選擇build或是用build好的檔案直接燒錄。(如果注重安全，應該自行build，release中應該會有提供對應版本的build。)

#### 燒錄build好的檔案

* 從 github 的 release 下載對應的燒錄好的 `.uf2` 檔案。
* 常按板子上的 boot 按鈕，按住的同時接上電腦。正常而言會彈出此板子的資料夾，可以把燒錄好的 `.uf2` 放到裡面。(如果沒彈出資料夾可以多試幾遍；或是你選擇用的線或電腦端口需要更換)
* `.uf2` 檔案放入後，板子會自己重啟。此時就能正常使用了。

#### 自己build

* 取得一塊 weshare-lcd-1.28 的板子(不要買可以觸控的，腳位和程式不同，要自行修改。weshare中文是微雪。)
* 有一根能傳輸且穩定的線，電腦有 usb 3.2(或同等傳輸速度)的接口。
* 跟隨微雪官方的wiki安裝 arduino IDE(需要安裝對應的raspberry pi模塊)，以及自行安裝 VScode 和裡面的插件 platformIO。
* 從 github 上，clone 此專案。
* 設定好後，就能直接點 vscode 右上角的打勾按鈕(platormIO 的 build)，直接 build。
* build 完後，會產生一個 release 的資料夾，裡面會有 `.uf2` 的固件檔案。
* 常按板子上的 boot 按鈕，按住的同時接上電腦。正常而言會彈出此板子的資料夾，可以把燒錄好的 `.uf2` 放到裡面。(如果沒彈出資料夾可以多試幾遍；或是你選擇用的線或電腦端口需要更換)
* `.uf2` 檔案放入後，板子會自己重啟。此時就能正常使用了。

### 代碼內可以修改的部分

* 液體相關的代碼
  * `lib\FluidRenderer` 資料夾中。`FluidRenderer.hpp` 裡面可以改液體粒子的常數。
* 休眠模式
  * `lib\LowPower` 裡面，會使用到 `LowPowerRP2040.h`，如果想要修改晃動喚醒和使用端口喚醒，可以在此更改。
* 晃動換色
  * 在液體相關的代碼裡面，以及 `main.cpp`裡面也有部分。

###  更好的處理

* 可選
  * 外殼
    * 下載此專案內的shell檔案，拿去3d打印外殼
  * 電池
    * 可以自行購買 mx1.25 端口的電池安裝或自行焊接。
  * 開關
    * 自行焊接開關，在代碼內指定對應的pin來休眠或喚醒，省電量。
