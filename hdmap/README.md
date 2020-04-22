# 工程说明文档

## 版本更新

#### hdmap v1.0_8.06_9.00

- 在test.cpp中整合深度估计的内容,可打开注释调试
- 修复read_hdmap.h中,中间结果查询的部分返回错误(exicode=139)

#### hdmap v1.0-8.04_14.50

- 底图数据读取加入了经纬度转东北天

#### hdmap v1.0-8_03_24.00

- 整合最新版的姜昊辰的RT计算代码,在test.cpp目录

> hdmap :项目工程总目录
>
> rviz可视化模块的代码可直接放在此目录下与hdmap平级
>
> 代码提交更新直接更新test里面的代码,最好自己封装成函数.
>
> 提交的个人部分的代码,需要有一定程度的注释方便理解.并用-----隔开-----
>
> 若更新变动较大,或者添加新的依赖库,需要找我说明

## 安装环境

若安装出现问题,可直接找同伴来调试

- Clion

  ```
  1.下载地址:https://www.jetbrains.com/clion/
  2.激活码地址:http://idea.lanyus.com/
  3.版本随意,最新版即可
  ```
  
- Protobuf

  ```
  1.下载地址:https://github.com/protocolbuffers/protobuf/releases
  2.解压之后,在目录下执行
   	./configure
       make
       make check
       sudo make install
       sudo ldconfig # refresh shared library cache.
  3.常用命令
  	protoc --cpp-out=.  *.proto   #生成proto文件对应的c++接口
  	protoc --decode-raw  xx.pb    #直接在终端里打印解码后的pb文件数据
  ```

## 使用

clone到本地之后直接用clion打开

安装protobuf,在此目录下执行 protoc --cpp-out=.  *.proto生成.h和.cc文件

在test.cpp main()函数作为程序的入口

各模块封装的库可以放在utils目录下

把data数据完整的拷贝进相应文件夹

