# Clone Ros-porting-tool
`git clone https://gitee.com/openeuler/ros-porting-tools.git`  
![克隆工具](./Screenshot_2024-12-22_07-11-42-1.png)

## 运行脚本

`get-repo-list.sh`  
![运行脚本](./Screenshot_2024-12-22_12-37-30-1.png)


##  安装vcs工具

由于官方源不存在这个工具，所以需要使用pip安装
'pip install vcstool'  
![安装工具](./Screenshot_2024-12-22_07-42-53-1.png)


## 下载ros源码到src目录下

创建src文件夹
'mkdir src'
下载源码,请使用out/*作为输出路径，因为脚本内环境变量是这么写的
`vcs import output/src < output/ros.repos `
![下载源码](./Screenshot_2024-12-22_14-10-54-1.png)

## 执行脚本  

get-pkg-src.sh
get-pkg-deps.sh
gen-pkg-spec.sh

![alt text](Screenshot_2024-12-22_14-33-07-1.png) 
![alt text](Screenshot_2024-12-22_14-20-18-1.png)

以上为这些脚本文件的使用流程，这些脚本文件使用会出现些许问题，下面介绍其中一部分脚本文件具体逻辑。

# 脚本文件

## base.h
1.读取环境变量：  
脚本首先从当前目录下的 config 文件中读取一系列的配置项，包括 ROS 分发版（ROS_DISTRO）、源码包的来源（SRC_TAR_FROM）、调试模式（DEBUG）、基础 URL（SRC_TAR_BASE_URL）、Gitee 组织和域名（GITEE_ORG 和 GITEE_DOMAIN）、OBS 域名和项目（OBS_DOMAIN 和 OBS_PROJECT）、OpenEuler 的基础版本和SP版本（OPENEULER_BASE_VERSION 和 OPENEULER_SP_VERSION）。  

2.设置分支名称：  
根据读取到的 ROS_DISTRO 和 OPENEULER_BASE_VERSION，设置 OpenEuler 开发分支（OPENEULER_DEV_BRANCH）、下一个版本的依赖分支（OPENEULER_ROS_DEP_NEXT_BRANCH）、多版本分支（OPENEULER_NEXT_BRANCH）。  
如果 OPENEULER_SP_VERSION 为空，则设置 OPENEULER_SP_BRANCH 和 OPENEULER_ROS_DEP_PKG_BRANCH 为不包含 SP 版本的分支名称；如果不为空，则包含 SP 版本。

3.创建目录结构：  
脚本创建了一系列的目录，包括输出目录（OUTPUT）、临时目录（ROS_OUTPUT_TMP）以及 ROS 相关的目录（ROS_SRC_BASE、ROS_DEPS_BASE、ROS_REPO_BASE、ROS_BB_BASE、ROS_OBS_BASE、ROS_GITEE_BASE）。  

4.日志函数： 
定义了三个日志函数：error_log、info_log 和 debug_log，用于记录错误、信息和调试日志。debug_log 函数只有在 DEBUG 模式开启时才会记录日志。  

5.检查 ROS 分发版：
脚本检查是否定义了 ROS_DISTRO。如果没有定义，会记录错误日志并退出脚本

## get-repo-list.sh
1.导入基础配置：
脚本首先通过 . base.sh 导入了另一个名为 base.sh 的脚本，这通常包含了一些基础的变量和函数定义。 

2.定义变量：
定义了一系列的变量，包括 ROS 项目列表文件、版本修复文件、ROS 仓库文件和 URL 等。   

3.定义函数 project_version_fix：
这个函数接受两个参数：项目名和原始版本号。  
如果 ROS_VERSION_FIX 文件不存在，则直接返回原始版本号。  
如果文件存在，则读取文件内容，查找与项目名匹配的条目，并返回对应的修复版本号。  
如果没有找到匹配的条目，则返回原始版本号。  

4.定义主函数 main：   
首先检查 ROS_PROJECTS_LIST 文件是否存在，如果不存在，则记录错误日志并退出。  
清空 ROS_PKG_LIST、ROS_REPOS_URL 和 ROS_PROJECTS_NAME 文件，准备写入新的数据。  
读取 ROS_PROJECTS_LIST 文件中的每一行，每行包含包名、URL、状态和版本信息。  
对于每一行，如果包名为空或者状态是 disabled 或 unknown，则跳过处理。  
分析 URL 以提取项目名、新 URL 和分支名（tree）。  
如果 URL 是 GitHub、GitLab、Gitee 或 Bitbucket 的，脚本会根据 URL 的格式提取项目名和分支名，并构造新的 Git 仓库 URL。  
如果分析失败（项目名、新 URL 或版本为空），则记录错误日志并退出。  
将包名、项目名、版本、Git 仓库 URL 和分支名写入 ROS_PKG_LIST 文件。  
检查 ROS_REPOS_URL 文件中是否已经存在新 URL，如果存在，则跳过。  
如果不存在，则将新 URL 添加到 ROS_REPOS_URL 文件，并在 ROS_REPOS 文件中添加对应的仓库配置信息。  
使用 project_version_fix 函数获取修复后的版本号，并写入 ROS_REPOS 文件。  
最后，记录信息日志，表示 ros.repos 文件已生成。  

**这一步有问题，后续大部分问题可能都源于这一步，这一步读取本地脚本文件的ros-porting-tools/ros/humble/ros-projects.list获取Ros项目名称，链接，版本。但是这个文件已经是两年前更新的。所以后续会出现很多文件无法找到的问题。**

## clone-projects-from-gitee.sh
1.导入基础配置：
脚本首先通过 . base.sh 导入了另一个名为 base.sh 的脚本，这通常包含了一些基础的变量和函数定义。

2.定义变量：
定义了 Gitee 的 URL（GITEE_URL），Gitee 基础目录（GITEE_BASE）和克隆分支（CLONE_BRANCH）。

3.定义 prepare 函数：
这个函数检查 ROS_PROJECTS_NAME 文件是否存在，如果不存在，则记录错误日志并退出。
创建 Gitee 基础目录。

4.定义 main 函数：
调用 prepare 函数进行准备工作。
记录开始克隆项目的信息日志。
读取 ROS_PROJECTS_NAME 文件中的每一行，每行包含一个项目名。
对于每个项目，检查是否已经克隆到本地，如果是，则拉取最新的分支。
如果项目尚未克隆，则克隆项目到 Gitee 基础目录。
检查项目中是否存在指定的分支，如果存在，则切换到该分支；如果不存在，则创建并切换到该分支。

## gen-pkg-spec.sh

1.导入基础配置：
脚本首先通过 . base.sh 导入了另一个名为 base.sh 的脚本，这通常包含了一些基础的变量和函数定义。

2.定义变量：
定义了 ROS 软件包源代码列表文件（ROS_PKG_SRC）、ROS 软件包修复文件路径（ROS_PACKAGE_FIX）和 ROS 软件包重映射文件（ROS_PKG_REMAP）。

3.定义 prepare 函数：
检查 ROS_SRC_BASE 是否定义和存在，检查 ROS_PKG_SRC 和 ROS_PKG_LIST 文件是否存在，如果条件不满足，则记录错误日志并退出。
如果 GEN_ONE 参数为空，则删除 ROS_REPO_BASE 目录，并创建 ROS_REPO_BASE 目录。

4.定义 spec_fix 函数：
用于修复软件包的 spec 文件，根据 spec_fix 文件夹中的规则删除或添加依赖。

5.定义 rename_requires 函数：
用于重命名 spec 文件中的依赖。

6.定义 replace_key_word 函数：
用于替换 spec 文件中的关键词汇。

7.定义 gen_requires 函数：
用于生成 spec 文件中的依赖部分。

8。定义 modify_spec 函数：
用于修改 spec 文件，包括版本、描述、许可证、URL、依赖等。

9.定义 spec_type_fix 函数：
用于根据软件包类型修复 spec 文件。

10.定义 package_fix 函数：
用于修复软件包，包括复制修复文件和替换关键词汇。

11.定义 main 函数：
执行准备工作，分析 ROS 软件包，生成 spec 文件，修复 spec 文件，并处理每个软件包。


## get-deps-src.sh

1.定义变量：
定义了 ROS 依赖列表文件（ROS_DEPS）和 ROS 依赖源代码列表文件（ROS_DEPS_SRC）。

2.定义 prepare 函数：
检查 ROS_DEPS_BASE 目录是否存在，如果不存在，则记录错误日志并退出。
检查当前操作系统是否为 Ubuntu，如果不是 Ubuntu 系统，则记录错误日志并退出。
清空 ROS_DEPS 和 ROS_DEPS_SRC 文件，准备写入新的依赖信息。

3.定义 main 函数：
调用 prepare 函数进行准备工作。
记录开始分析 ROS 软件包的信息日志。
遍历 ROS_DEPS_BASE 目录下所有以 ExtDeps 结尾的文件，这些文件包含了 ROS 软件包的外部依赖。
对于每个依赖，记录分析信息，并尝试使用 apt show 命令获取软件包信息。
如果 apt show 命令失败（即软件包不存在），则将依赖名称记录到 ROS_DEPS_SRC 文件中。
如果 apt show 命令成功，提取软件包的源代码包名称，并记录到 ROS_DEPS_SRC 文件中。

**此脚本文件仅可在Ubuntu系统上运行！！！**

## get-pkg-deps.sh

1.定义变量：
定义了 ROS 软件包源代码列表文件（ROS_PKG_SRC）。

2.定义 prepare 函数：
检查 ROS_SRC_BASE 是否定义和存在，检查 ROS_PKG_LIST 和 ROS_PKG_SRC 文件是否存在，如果条件不满足，则记录错误日志并退出。
清空 ROS_DEPS_BASE 目录下的文件。

3.定义 write_dep 函数：
用于向指定的依赖文件中写入新的依赖项。

4.定义 gen_depend 函数：
用于生成 ROS 软件包的依赖关系。
从 package.xml 文件中提取依赖项，并根据依赖类型（如 build_depend、exec_depend 等）生成相应的依赖列表。

5.定义 main 函数：
调用 prepare 函数进行准备工作。
遍历 ROS_PKG_SRC 文件中的每一行，每行包含软件包名、路径和版本。
对于每个软件包，检查 package.xml 文件是否存在。
调用 gen_depend 函数生成各种类型的依赖关系。
记录生成依赖关系的信息日志

## get-pkg-src.sh  

1.定义变量：
定义了 ROS 软件包源代码列表文件（ROS_PKG_SRC）。

2.定义 prepare 函数：
检查 ROS_SRC_BASE 是否定义和存在，检查 ROS_PKG_LIST 文件是否存在，如果条件不满足，则记录错误日志并退出。
清空 ROS_PKG_SRC 文件，准备写入新的软件包源代码路径信息。

3.定义 find_pkg_src_path_by_package_xml 函数：
用于在给定的路径下搜索包含特定 <name> 标签的 package.xml 文件，以确定软件包的源代码路径。

4.定义 main 函数：
调用 prepare 函数进行准备工作。
记录开始分析 ROS 软件包的信息日志。
遍历 ROS_PKG_LIST 文件中的每一行，每行包含软件包名、基础路径、版本、Git URL 和分支。
对于每个软件包，检查是否为 Ubuntu 源代码包（通过 SRC_TAR_FROM 变量判断）。
如果是 Ubuntu 源代码包，提取软件包的原始名称和版本，并尝试找到对应的 package.xml 文件。
如果不是 Ubuntu 源代码包，尝试找到对应的 package.xml 文件，使用 find_pkg_src_path_by_package_xml 函数辅助查找。
如果找到 package.xml 文件，将软件包名、源代码路径、版本、Git URL 和分支写入 ROS_PKG_SRC 文件。
如果没有找到源代码路径，记录错误日志。

## get-src-from-github.sh

用于从 ros.repos 文件中描述的版本控制系统（VCS）仓库下载 ROS（Robot Operating System）软件包到本地目录。

## push-projects-to-gitee.sh

用于将 ROS（Robot Operating System）项目的代码从本地仓库推送到 Gitee 上的远程仓库。

1.定义变量：
定义了用于存储推送项目的列表文件（ROS_PUSH_LIST）。

2.定义 prepare 函数：
检查 ROS_PROJECTS_NAME 文件是否存在，如果不存在，则记录错误日志并退出。
创建 ROS_GITEE_BASE 目录，用于存储 Gitee 远程仓库的本地副本。
清空 ROS_PUSH_LIST 文件，准备记录需要推送的项目。

3.定义 main 函数：
调用 prepare 函数进行准备工作。
记录开始推送项目到 Gitee 的信息日志。
遍历 ROS_PROJECTS_NAME 文件中的每一行，每行包含一个项目名。
对于每个项目，检查对应的本地 Git 仓库是否存在。
如果项目不存在，则记录错误日志并跳过。
如果项目存在，切换到项目目录，重置工作目录（删除所有未跟踪文件），并从 ROS_REPO_BASE 复制文件到 ROS_GITEE_BASE。
检查项目是否有修改、未跟踪文件或删除文件。
如果没有变化，则跳过当前项目。
如果有变化，将项目名添加到 ROS_PUSH_LIST 文件，并记录开始推送项目的信息日志。
添加所有变化到 Git，并根据 PUSH 参数决定是否进行提交和推送。
如果 PUSH 参数不是 "yes"，则只进行差异检查，不推送。
如果 PUSH 参数是 "yes"，则提交更改，并推送到 Gitee 远程仓库。
如果推送失败，则记录错误日志并跳过当前项目。