# IMUStudio · C++

AT32F405 / LSM6DSV16X 串口上位机。Windows 10/11 x64 双击 `IMUStudio.exe` 即可运行。EXE 内置 SQLite、板体模型和图标。

## 使用

- 串口默认 2 Mbps、8N1。连接后查询状态与诊断。配置先写入 RAM，确认后可显式保存到 Flash。
- 实时页同时显示波形和三维姿态。波形页点击图例切换通道，滚轮缩放时间，Ctrl + 滚轮缩放纵轴，拖动平移。三维视图支持拖动和滚轮。
- 标定与诊断页可采集姿态、查询参数和查看阶段计时。字段和命令沿用原 Python 上位机的协议 v1。
- 日志页兼容旧版 SQLite `.imulog`，可限时段导出数据和事件 CSV。回放打开后，在实时或波形页查看数据。
- 右上角检查更新。找到新版时可下载，SHA-256 校验成功后点击“重启更新”。EXE 需要放在当前用户可写的目录。

## 构建

在本目录运行。需 CMake 3.24+、Visual Studio 2022 的 C++ 工作负载及 Windows SDK，首次配置下载并校验构建依赖。

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

产物为 `build/Release/IMUStudio.exe`。本地 MinGW-w64 也可用 Ninja 的 Release 配置构建。

## GitHub 发布

仓库的 IMUStudio 工作流在源码变更时构建、测试和上传单个 EXE。发布时将 `CMakeLists.txt` 中的版本与 Git 标签保持一致，例如版本 `1.0.1` 对应 `imustudio-v1.0.1`。将该标签推送到 GitHub 后，工作流通过测试并核对 EXE 版本，再创建 Release。

更新器查询 `ZhangStudyLife/AT32F405_IMU` 的正式 IMUStudio Release，选择比当前版本新的 `IMUStudio.exe`，并使用 GitHub 提供的资源摘要校验。CI 从所在仓库写入更新地址，本地可用 `-DIMUSTUDIO_REPOSITORY=owner/repo` 指定公开仓库。

板体网格沿用旧版 `../imustudio/assets/board.npz` 中由 AT32F405_IMU.SLDPRT 转出的 652 个三角形，保留传感器坐标，改用灰阶。SQLite 为公有领域，nlohmann/json 的 MIT 许可嵌入 EXE 并附在 `assets/JSON-LICENSE.txt`。
