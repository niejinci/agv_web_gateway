// ==========================================
// 1. Three.js 3D 场景初始化
// ==========================================
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 5000);
camera.position.set(0, 15, 15);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);
const controls = new THREE.OrbitControls(camera, renderer.domElement);

// 网格地面
// 200米 x 200米的大小，划分为 100 个格子 (每个格子 2x2 米)
const gridHelper = new THREE.GridHelper(200, 100, 0x444444, 0x222222);
gridHelper.rotation.x = Math.PI / 2;
scene.add(gridHelper);

// ROS 坐标系转换 (Z 轴朝上)
// Three.js (前端呈现)：默认 Y 轴是朝上的（天空），Z 轴是朝向你的（深度）。
// ROS / AGV 物理底盘 (后端数据)：通常使用另一个标准的右手系，即 Z 轴朝上（天空），X 轴朝向车头正前方，Y 轴朝向车身左侧。
// 如果不做转换，AGV 发过来的点云直接画在 Three.js 里，原本平铺在地面（XY平面）的地图，在浏览器里就会像一堵“墙”一样竖在屏幕上。
// 解决方案：在 Three.js 场景里创建一个父级容器（Group），把它旋转 -90 度（-Math.PI/2），让它的 Z 轴朝上。然后所有从 ROS 发过来的坐标数据都放在这个容器里，自然就完成了坐标系的转换。
const rosRoot = new THREE.Group();
// 绕 X 轴旋转 -90 度
// 这会把原本朝上的 Y 轴拍下去，把原本朝向你的 Z 轴立起来当成天空，做完这个旋转后，底盘发过来的 X 和 Y 就会老老实实地趴在 Three.js 的地平面上了
rosRoot.rotation.x = -Math.PI / 2;
scene.add(rosRoot);

// ==========================================
// 1.5 【修改】：移除原来写死的 PCD 加载代码，改为动态全局变量
// ==========================================
let currentMapPoints = null; // 用于记录当前加载的地图，方便切换时清理旧地图

// --- 创建各种图形容器与材质 ---
// 1. 小车坐标容器 (代表小车中心)
const agvGroup = new THREE.Group();
rosRoot.add(agvGroup);
const axesHelper = new THREE.AxesHelper(1.5); // 增加红绿蓝坐标轴代表小车方向 (红=X前, 绿=Y左)
agvGroup.add(axesHelper);

function createTextSprite(message, color) {
    const canvas = document.createElement('canvas');
    canvas.width = 128;
    canvas.height = 128;
    const context = canvas.getContext('2d');
    context.font = "Bold 60px Arial";
    context.fillStyle = color;
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText(message, 64, 64);

    const texture = new THREE.CanvasTexture(canvas);
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture, transparent: true });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(0.6, 0.6, 1); // 调整文字大小
    return sprite;
}

// 在轴的末端稍微往外一点的位置 (1.7) 放上文字
const spriteX = createTextSprite('X', '#ff4444');
spriteX.position.set(1.7, 0, 0);
agvGroup.add(spriteX);

const spriteY = createTextSprite('Y', '#44ff44');
spriteY.position.set(0, 1.7, 0);
agvGroup.add(spriteY);

const spriteZ = createTextSprite('Z', '#44aaff');
spriteZ.position.set(0, 0, 1.7);
agvGroup.add(spriteZ);

// 2. Point Cloud (全局点云 - 灰色)
const matPcl = new THREE.PointsMaterial({ color: 0xaaaaaa, size: 0.2 });
const meshPcl = new THREE.Points(new THREE.BufferGeometry(), matPcl);
meshPcl.frustumCulled = false; // 【核心修复】：关闭视锥体剔除！强制每一帧都渲染
rosRoot.add(meshPcl);

// 3. Scan2PointCloud (避障点云 - 红色)
const matScan2Pcl = new THREE.PointsMaterial({ color: 0xff0000, size: 0.08 });
const meshScan2Pcl = new THREE.Points(new THREE.BufferGeometry(), matScan2Pcl);
meshScan2Pcl.frustumCulled = false; // 【核心修复】：关闭视锥体剔除！强制每一帧都渲染
rosRoot.add(meshScan2Pcl);

// 4. Obstacle Pcl (障碍物点云 - 橙色)
const matObstPcl = new THREE.PointsMaterial({ color: 0xffaa00, size: 0.08 });
const meshObstPcl = new THREE.Points(new THREE.BufferGeometry(), matObstPcl);
meshObstPcl.frustumCulled = false; // 【核心修复】：关闭视锥体剔除！强制每一帧都渲染
rosRoot.add(meshObstPcl);

// 5. Model Polygon (小车模型轮廓 - 蓝色线条)
const matModelPoly = new THREE.LineBasicMaterial({ color: 0x00aaff, linewidth: 2 });
const meshModelPoly = new THREE.LineLoop(new THREE.BufferGeometry(), matModelPoly);
meshModelPoly.frustumCulled = false; // 【核心修复】：关闭视锥体剔除！强制每一帧都渲染
rosRoot.add(meshModelPoly);

// 6. Obstacle Polygon (避障轮廓 - 品红色线条)
const matObstPoly = new THREE.LineBasicMaterial({ color: 0xff00ff, linewidth: 2 });
const meshObstPoly = new THREE.LineLoop(new THREE.BufferGeometry(), matObstPoly);
meshObstPoly.frustumCulled = false; // 关闭剔除，确保始终渲染
rosRoot.add(meshObstPoly);


// ==========================================
// 2. WebSocket 与 UI 控制逻辑
// ==========================================
// 写死连接本机的 8080
// const ws = new WebSocket('ws://127.0.0.1:8080');
const ws = new WebSocket(`ws://${window.location.host}`);
const statusDiv = document.getElementById('status');
const logOutput = document.getElementById('log-output');

// 维护各个数据流的开关状态 (默认 false)
const streamState = {
    point_cloud: false,
    agv_position: false,
    obst_polygon: false,
    model_polygon: false,
    scan2pointcloud: false,
    obst_pcl: false
};

ws.onopen = () => {
    statusDiv.textContent = '已连接到网关 (127.0.0.1:8080)';
    statusDiv.classList.add('connected');

    // 连上网关后，立刻请求底层的地图列表！
    ws.send("get_map_list");
};

// 【新增】：绑定加载地图按钮点击事件
document.getElementById('btn-load-map').onclick = () => {
    const selected = document.getElementById('map-select').value;
    if (!selected) {
        alert("请先选择一个地图！");
        return;
    }

    // 1. 如果之前已经加载过地图，先从场景中移除���释放内存
    if (currentMapPoints) {
        rosRoot.remove(currentMapPoints);
        currentMapPoints.geometry.dispose();
        currentMapPoints.material.dispose();
        currentMapPoints = null;
    }

    // 2. 拼装动态请求的 URL (例如: /pcd/pc/SS27)
    let pcdUrl = `/pcd/${selected}`;
    // 【新增】：如果勾选了强制更新，就在 URL 后面挂一个随机时间戳！
    const forceUpdate = document.getElementById('cb-force-update').checked;
    if (forceUpdate) {
        pcdUrl += `?t=${new Date().getTime()}`; // 例如：/pcd/pc/SS27?t=1700000000000
    }
    console.log("准备下载并加载地图: " + pcdUrl);

    // 3. 开始加载新地图
    const pcdLoader = new THREE.PCDLoader();
    pcdLoader.load(
        pcdUrl,
        function (points) {
            points.material.color.setHex(0x555555); // 深灰色
            points.material.size = 0.2;
            rosRoot.add(points);
            currentMapPoints = points; // 记录当前地图
            console.log("✅ 地图加载成功！点数:", points.geometry.attributes.position.count);
        },
        function (xhr) {
            console.log((xhr.loaded / xhr.total * 100).toFixed(2) + '% loaded');
        },
        function (error) {
            console.error('地图加载出错:', error);
            alert("加载失败：可能该地图没有对应的 .pcd 3D点云文件！");
        }
    );
};

ws.onclose = () => {
    statusDiv.textContent = '连接已断开';
    statusDiv.classList.remove('connected');
};

// 绑定所有的开关按钮
Object.keys(streamState).forEach(key => {
    const btn = document.getElementById(`btn-${key}`);
    // 【修改】：初始化按钮文本为 Start，去掉红色的 stop-btn 类
    btn.textContent = 'Start';
    btn.classList.remove('stop-btn');

    btn.onclick = () => {
        streamState[key] = !streamState[key]; // 切换状态
        if (streamState[key]) {
            ws.send(`start_${key}`);
            btn.textContent = 'Stop';
            btn.classList.add('stop-btn');
        } else {
            ws.send(`stop_${key}`); // 发送停止指令
            btn.textContent = 'Start';
            btn.classList.remove('stop-btn');

            // 可选：停止时清空现有的图形数据
            clearGeometry(key);
        }
    };
});

// 绑定获取日志列表按钮
document.getElementById('btn-get_log').onclick = () => {
    logOutput.value = "请求中...";
    ws.send("get_log_list");
};

let isFirstPosition = true;

// ==========================================
// 3. WebSocket 数据解析与渲染
// ==========================================
ws.onmessage = (event) => {
    try {
        const msg = JSON.parse(event.data);
        const type = msg.type;
        const res = msg.payload;

            // 【新增】：处理底层返回的地图列表
        if (type === 'map_list') {
            if (res.code === 0 && res.data) {
                const mapSelect = document.getElementById('map-select');
                mapSelect.innerHTML = ''; // 清空原有选项

                // 遍历解析返回的 JSON (如 pc, rcs 分类)
                for (const category in res.data) {
                    const mapFiles = res.data[category];
                    mapFiles.forEach(fileName => {
                        // 把 "SS27.smap" 去掉后缀，变成 "SS27"
                        const mapName = fileName.replace('.smap', '');
                        const option = document.createElement('option');
                        // value 存成 "pc/SS27" 的格式，方便后端解析
                        option.value = `${category}/${mapName}`;
                        option.textContent = `[${category}] ${mapName}`;
                        mapSelect.appendChild(option);
                    });
                }
                console.log("地图列表更新完毕");
            } else {
                console.warn("获取地图列表失败:", res.message);
            }
            return;
        }

        if (res.code !== 0) {
            console.warn(`[${type}] 错误:`, res.message);
            return;
        }

        const data = res.data;

        // 根据 type 更新对应的 BufferGeometry
        switch (type) {
            case 'point_cloud':
                updatePointCloud(meshPcl.geometry, data); // data 是个数组
                break;
            case 'scan2pointcloud':
                updatePointCloud(meshScan2Pcl.geometry, data.points);
                break;
            case 'obst_pcl':
                updatePointCloud(meshObstPcl.geometry, data.points);
                break;
            case 'model_polygon':
                updatePolygon(meshModelPoly.geometry, data.polygon.points);
                break;
            case 'obst_polygon':
                updatePolygon(meshObstPoly.geometry, data.polygon.points);
                break;
            case 'agv_position':
                // 1. 更新小车本体在 ROS 坐标系下的位置
                agvGroup.position.set(data.x, data.y, data.z);
                agvGroup.rotation.z = data.theta; // 偏航角

                // 2. 获取小车在 Three.js 世界坐标系下的绝对位置
                let worldPos = new THREE.Vector3();
                agvGroup.getWorldPosition(worldPos);

                // 3. 【全新升级：完美的视角跟随逻辑】
                if (isFirstPosition) {
                    // 第一次：直接把相机搬到小车上方 15 米处
                    camera.position.set(worldPos.x, worldPos.y + 15, worldPos.z + 15);
                    controls.target.copy(worldPos);
                    controls.update(); // 必须调用 update 让控制器应用新焦点
                    isFirstPosition = false;
                } else {
                    // 后续帧：获取页面上复选框的状态
                    const cbFollow = document.getElementById('cb-follow');
                    if (cbFollow && cbFollow.checked) {
                        // 航拍平移算法核心：
                        // a. 计算出目前相机和当前焦点的“相对偏移量 (offset)”
                        const offset = new THREE.Vector3().subVectors(camera.position, controls.target);

                        // b. 把焦点移动到小车的新位置
                        controls.target.copy(worldPos);

                        // c. 把相机也同步移动（新焦点位置 + 刚才的偏移量）
                        camera.position.copy(worldPos).add(offset);

                        controls.update();
                    }
                    // 如果没有勾选，则什么也不做，让用户自由漫游
                }
                break;
            case 'log_list':
                // 格式化输出 JSON
                logOutput.value = JSON.stringify(data, null, 2);
                break;
        }
    } catch (e) {
        console.error("解析消息失败:", e, event.data);
    }
};

// 辅助函数：高效更新点云 Geometry
function updatePointCloud(geometry, pointsArray) {
    if (!pointsArray || pointsArray.length === 0) return;
    const positions = new Float32Array(pointsArray.length * 3);
    for (let i = 0; i < pointsArray.length; i++) {
        positions[i*3] = pointsArray[i].x;
        positions[i*3 + 1] = pointsArray[i].y;
        positions[i*3 + 2] = pointsArray[i].z;
    }
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.attributes.position.needsUpdate = true;
}

// 辅助函数：高效更新多边形 Geometry
function updatePolygon(geometry, pointsArray) {
    if (!pointsArray || pointsArray.length === 0) return;
    const positions = new Float32Array(pointsArray.length * 3);
    for (let i = 0; i < pointsArray.length; i++) {
        positions[i*3] = pointsArray[i].x;
        positions[i*3 + 1] = pointsArray[i].y;
        positions[i*3 + 2] = pointsArray[i].z;
    }
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.attributes.position.needsUpdate = true;
}

// 辅助函数：清空数据
function clearGeometry(type) {
    const empty = new Float32Array(0);
    if (type === 'point_cloud') meshPcl.geometry.setAttribute('position', new THREE.BufferAttribute(empty, 3));
    if (type === 'scan2pointcloud') meshScan2Pcl.geometry.setAttribute('position', new THREE.BufferAttribute(empty, 3));
    if (type === 'obst_pcl') meshObstPcl.geometry.setAttribute('position', new THREE.BufferAttribute(empty, 3));
    if (type === 'model_polygon') meshModelPoly.geometry.setAttribute('position', new THREE.BufferAttribute(empty, 3));
    if (type === 'obst_polygon') meshObstPoly.geometry.setAttribute('position', new THREE.BufferAttribute(empty, 3));
}

// ==========================================
// 4. 渲染循环
// ==========================================
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}
animate();

window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// ==========================================
// 5. 面板折叠逻辑
// ==========================================
const btnToggle = document.getElementById('btn-toggle-panel');
const panelContent = document.getElementById('panel-content');
let isPanelOpen = true;

btnToggle.onclick = () => {
    isPanelOpen = !isPanelOpen;
    if (isPanelOpen) {
        panelContent.style.display = 'block';
        btnToggle.textContent = '折叠 -';
    } else {
        panelContent.style.display = 'none';
        btnToggle.textContent = '展开 +';
    }
};