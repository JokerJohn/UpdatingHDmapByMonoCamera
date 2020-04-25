整个项目用到了google protobuf，在运行前需要装好protobuf。

具体步骤如下：

```
1.下载地址:https://github.com/protocolbuffers/protobuf/releases
2.解压之后,在目录下执行
 	./configure
     make
     make check
     sudo make install
     sudo ldconfig # refresh shared library cache.
     
     # python版也需要安装
     # 验证
     protoc --version
     out:libprotoc 3.5.1 #表示安装成功

    # 安装python protobuf模块
    cd python
    python setup.py install # 这一步需要先安装six模块才能正常安装
     
3.常用命令
	protoc --cpp-out=.  *.proto   #生成proto文件对应的c++接口
	protoc --python_out=.  *.proto #生成proto文件对应的py接口
	protoc --decode-raw  xx.pb    #直接在终端里打印解码后的pb文件数据
```

