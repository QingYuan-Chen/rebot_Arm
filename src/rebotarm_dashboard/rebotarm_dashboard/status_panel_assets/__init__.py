"""面板静态资源的资源包标识模块(不导出任何符号).

存在的意义
    让 HTML/JS/CSS 等页面资源位于一个可导入的子包内,从而能用 importlib.resources
    以"包资源"的方式读取(见同包的 status_panel_page.py),不依赖源码树的相对路径.

打包
    随包分发的具体文件清单在 setup.py 的 package_data 中声明(当前为 index.html);
    新增资源文件时必须同步更新该清单,否则安装后读取会失败.
"""
