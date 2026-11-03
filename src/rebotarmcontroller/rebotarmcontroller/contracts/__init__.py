"""控制器跨组件数值与能力接口。

文件职责：声明 contracts 子包，不汇总导入或创建业务状态。
对外接口：从 controller_contracts/controller_values 取值类型，从 component_ports/ros_ports 取能力声明。
安全约束：契约不访问设备、不创建线程；冻结端口不替代调用方的仲裁和安全检查。
文件结构：本文件仅声明用途，四个模块按值契约与调用能力分工。
"""
