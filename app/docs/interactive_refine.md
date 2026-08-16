# 交互操作界面的优化

## 1. TCP offset的数据不从界面设置，可以直接从urdf读取，如果urdf中没有则不需要使用TCP数据；

## 2. 不同的模版之间切换很慢，而且图像和路径显示不同步，比如从154353切换到151323，显示不同步，也很慢；

## 3. 2D图像上的路径不需要用不同颜色区分，可以在每条路径的起点使用编号就可以；

## 4. 右边区域的按钮不需要用不同颜色的区分，只需要跟Capture保持同样的颜色就可以；

## 5. TCP的路径（包括 Manual TCP和TCP Opt两个页面功能）生成的文件要修改， Manual生成的为scan.manual.raw.paths.yaml，优化生成的为scan.manual.opt.paths.yaml；同时也对应要修改report.json，TCP诊断和优化的页面需要修改：每一种优化过后的每一个waypoint、最大关节速度，都需要显示 3 组信息对比， raw/opt/poi, 其中 poi 参考第6 节的设计，同时也需要在页面中显示出来;

## 6.  POI支持，这是最大的改动，增加一个新功能：
### 1. 对所有路径的waypoint点做姿态约束，约束条件在执行前设置，指定一个固定姿态（默认从home获取，也可以人工在页面指定），允许有一定姿态容差（可设置，Rx,Ry,Rz在固定姿态上的容差范围，比如Rx为±3°，Ry为±15°， Rz为±180°），然后优化的时候，把这个姿态作为每个waypoint点的约束，进行优化，这样可以保证机器人喷涂时姿态一致；

### 2. 实际上每一条路径有3个状态信息：姿态约束优化、普通优化（无姿态约束）、原始状态。在诊断页面上按第5条的方式在3个地方都要有信息（report.json文件，优化详情页，2D图像的 Tips上, 实际上有 3 个 report.json，raw.report.json，opt.report.json，poi.report.json,和 3 个path.yaml 文件， raw.path.yaml，opt.path.yaml，poi.path.yaml），实际上只有 raw.report.json 和 raw.path.yaml 是原始的， opt.report.json，poi.report.json, opt.path.yaml，poi.path.yaml 是优化生成的,可以通过不同的颜色来区分这三种状态，比如：姿态约束为绿色，普通优化为蓝色，原始状态为灰色,原始状态实际上是不需要有优化详情页和raw.path.yaml文件的，只需要raw.report.json文件；其中 path.yaml是发给真实机械臂和仿真机械臂执行（见第 7 节）的；

### 3. 在页面右上角Raw/Opt切换时需要增加一个姿态约束的选项，可以用一个英文的缩写, 最好是3个字母比如POI？Raw/Opt/POI ?

### 4. 文件列表中的仿真类型和机械臂执行类型的文件比如report.json/path.yaml等文件可以换一个更有意义的图标；

## 7. 支持简单的仿真，支持单路径、多路径、全部路径，右键点击path.yaml文件（可以选择不同的文件执行，比如raw.path.yaml，opt.path.yaml，poi.path.yaml），选择路径进行仿真，根据之前设定的速度把关节角度发送到 3D机械臂仿真区域执行；并且实时获取机械臂的位置同时把 3D的位姿映射到现在2D的图像区域中用闪动的小圆实时显示出来，并且把当前路径已经走过的部分换一个颜色标示出来，表示已经走过

