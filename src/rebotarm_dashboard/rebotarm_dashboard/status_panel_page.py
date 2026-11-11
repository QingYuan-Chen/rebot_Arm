"""面板首页 HTML 的加载入口:从包内资源读取页面文本并暴露为常量.

加载方式
    走 ``importlib.resources`` 按包资源读取,而不是相对文件路径或内联字符串,这样无论是
    源码树运行还是安装后的 ``site-packages``/符号链接布局都能定位到同一份页面;HTML 与
    后端代码分离,改版式无需触碰节点逻辑.

对外接口
    ``HTML_PAGE``:模块导入时即读取一次并缓存的结果,内容为 UTF-8 文本,由面板节点的
    HTTP 层直接作为响应体返回.读取失败(资源缺失或未随包安装)会在导入阶段直接报错,
    属于必须暴露的部署错误,不做静默降级.
"""

from __future__ import annotations

import importlib.resources


# 页面模板随包分发,见 setup.py 的 package_data;模块级只读一次,运行期不再重复访问磁盘.
HTML_PAGE = (
    importlib.resources.files("rebotarm_dashboard.status_panel_assets")
    .joinpath("index.html")
    .read_text(encoding="utf-8")
)
