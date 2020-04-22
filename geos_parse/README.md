# hdmap_pub
下有三个功能包
* hd_map(python)  
	读取hdmap.pb以自定义消息类型发出   	divider,trafficlight,lanemark三个topic  
* hdmap_viewer(C++,pcl)  
接收上述topic，并连线用pcl.visualizer显示出来  
* pcl_online_viewer(C++,pcl)  
一个接收点云topic用pcl.visualizer可视化的demo  
** 点云更新还会有id冲突，待改**  
# proto_test
C++读取pb文件
# prototest
使用C++与google protobuff
* 读prototxt
* 更改prototxt内元素内容
* 增加prototxt元素
* 删除prototxt元素
* 保存新的prototxt
# geostest
C++   
是geos库的一个可以跑通的demo

