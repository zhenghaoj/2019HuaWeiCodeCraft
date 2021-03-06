## 2019HuaWeiCodeCraft

武长赛区：闲云野鹤队，华中科技大学机械学院

队员：郑豪杰，邹华平，徐长斌

初赛24名；复赛武长赛区第3，决赛季军（8进4碰见冠军（咕咕咕），是在下输了！）

## 算法思想

* 根据地图最多能够容纳的车辆数即sum(roads[i].length * roads[i].channel)乘以一个系数作为地图不死锁情况下能够容纳车辆的最大数量范围的初始估算值range
* 每一时刻按照range减去地图上现有的车辆数来控制下一时刻的全图发车量need_car
* 车辆按照预置车辆、非预置优先车辆、非预置普通车辆分为三个车库，一旦车辆的planTime等于sys_time即可以发车，就把车辆放入车库中
* 发车时，预置车辆由于不能更改时间，到时间必须出生，优先发非预置优先车辆，其次非预置普通车辆，同优先级按照planTime主序，speed次序
* 每次车辆出生从车库中选取2 * need_car数量的车，根据拥堵情况规划路径，从中选取need_car数量的相比更不拥堵的车辆出生
* 车辆行驶过程中会间隔一定距离进行一次路径规划
* 不解死锁，通过不断变步长改变全图车的容纳量，迭代出更优解
* 当range大到一定程度时很多无解，获取最好的解，设置range_keep、time_keep保持优先车辆跑完的时间内不死锁，增大后面时间片range可以降低调度时间100左右
* 路径规格采用广度优先搜索，60000辆车规划2.5s可完成
* 统计道路拥堵包括道路上已有车辆，以及已经出生在道路中还没有上路的车辆，统计车辆所在道路以及将要经过的道路，同一辆车的路径，权重越往后越小，这个思想类似于图像匹配中的sift算子 + 高斯分布加权
* 玄学参数
 
## 判题器

* 决赛判题器运行时间20s左右(加了选车时路径规划和路上实时规划，所以比较慢)，前前后后重构了三遍，终于跟官方一样了

## 需求更改

* 复赛：只能改10%预置车辆的路径，我们选择固定更改最短路径和现有路径相差比较大的10%更改，复赛现场一直出bug，最后十分钟改好，没敢用
* 决赛：康康老师提前在群里说过给一些人绝地翻盘的机会，提前压了一波赛题，猜中了更改时间，提前想好了策略：10%全部改预置车辆中时间靠后的优先车辆的时间为planTime,最后40分钟开始改代码，只剩余5分钟改完bug,惊心动魄！！！上传调度时间减少了50左右，没有时间调参了，最后32进16成绩2111

## 参赛感悟

* 初赛判题器一直有一点差距，后来思考觉得应该是发车时不能上路车辆处理不同导致；想当然的认为cross的四个路就是按照东西南北排列，导致初赛当天判题器一直崩溃，想当然的认为id会连续，官方不会给自己找麻烦，我们低估了康康老师和峰峰老师，抢救代码到下午一点中，搞得异常心类，最后随便调了调参数，还剩余一个小时时间，觉得成绩还行，直接回宿舍睡大觉去了，最后成绩24名，好险。
* 复赛判题器终于一模一样了，一直在测想法，想了很多策略，有好多理论上可行，实际上不适合我们已有的模型，搞得我们真是心累。比如：给优先车辆让路，穿越路口加惩罚权等等。
* 决赛最后几天真是调不动了，结果直接把拥堵系数从10变成100，调度时间就降了150，决赛地图真是玄学，最后复盘看到康康老师的心型地图、各种中心城区空心图，我表示想哭。
* 关于过拟合的想法：在比赛地图已知情况下，过拟合一点无所谓，如果数据不可知，尽量不要过拟合。
* 最后感谢华为提供的平台，感谢队友的付出，今年终于有个奖了。ps:去年决赛第十，菜鸡眼睁睁看着各位大佬抱走大奖

## 关于代码需要改进的地方

* 由于前期队友之间没有沟通清楚，变量命名和函数命名规则用了下划线和小驼峰两种方法，看起来很不舒服（我有强迫症。。。）
* 有些函数参数过多
* 代码其实有很多冗余的地方，最后时间太紧张了，没有时间优化了

## 题外话

* 今日开源，正值巴萨被利物浦逆转之际，身为资深诺坎普球迷，心情十分沉重，心疼梅老板整个赛季这么拼命，你若有心安慰我受伤的心灵，请赐我一个star吧！

