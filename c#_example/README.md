Xema系列相机C#API
C++/CLI（Common Language Infrastructure）是一种特殊的C++语言，它支持在.NET平台上进行开发和编译。C++/CLI 可以让开发人员使用 C++语言来编写.NET应用程序，并且可以直接访问和使用.NET框架中的类库。其支持托管代码和非托管代码：C++/CLI中既可以编写非托管代码，也可以编写托管代码。这使得开发人员可以利用现有的C++代码库，并将其集成到.NET应用程序中。

环境搭建：VS2019，框架：.NET Framework 4.7.2

1、简单使用
在该路径下运行：./example --ip 192.168.××.××

2、API调用
打开编译器，如VS2019，新建C#"空项目(.NET Framework)"的工程，添加本路径下的example.cs的文件，并添加example.cs代码，将配置管理器改为Releasex64,右键"引用"->"添加引用"->"浏览"，添加Open_cam3d.dll文件，编译即可运行example示例。

