// ==========================================
// 1. Three.js 3D 场景初始化
// ==========================================
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 15, 15);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);
const controls = new THREE.OrbitControls(camera, renderer.domElement);

// 网格地面
const gridHelper = new THREE.GridHelper(50, 50, 0x444444, 0x222222);
gridHelper.rotation.x = Math.PI / 2;
scene.add(gridHelper);

// ROS 坐标系转换 (Z 轴朝上)
const rosRoot = new THREE.Group();
rosRoot.rotation.x = -Math.PI / 2;
scene.add(rosRoot);

// --- 创建各种图形容器与材质 ---
// 1. 小车坐标容器 (代表小车中心)
const agvGroup = new THREE.Group();
rosRoot.add(agvGroup);
const axesHelper = new THREE.AxesHelper(1.5); // 增加红绿蓝坐标轴代表小车方向 (红=X前, 绿=Y左)
agvGroup.add(axesHelper);

// 2. Point Cloud (全局点云 - 灰色)
const matPcl = new THREE.PointsMaterial({ color: 0xaaaaaa, size: 0.05 });
const meshPcl = new THREE.Points(new THREE.BufferGeometry(), matPcl);
rosRoot.add(meshPcl);

// 3. Scan2PointCloud (避障点云 - 红色)
const matScan2Pcl = new THREE.PointsMaterial({ color: 0xff0000, size: 0.08 });
const meshScan2Pcl = new THREE.Points(new THREE.BufferGeometry(), matScan2Pcl);
rosRoot.add(meshScan2Pcl);

// 4. Obstacle Pcl (障碍物点云 - 橙色)
const matObstPcl = new THREE.PointsMaterial({ color: 0xffaa00, size: 0.08 });
const meshObstPcl = new THREE.Points(new THREE.BufferGeometry(), matObstPcl);
rosRoot.add(meshObstPcl);

// 5. Model Polygon (小车模型轮廓 - 蓝色线条)
const matModelPoly = new THREE.LineBasicMaterial({ color: 0x00aaff, linewidth: 2 });
const meshModelPoly = new THREE.LineLoop(new THREE.BufferGeometry(), matModelPoly);
rosRoot.add(meshModelPoly);

// 6. Obstacle Polygon (避障轮廓 - 品红色��条)
const matObstPoly = new THREE.LineBasicMaterial({ color: 0xff00ff, linewidth: 2 });
const meshObstPoly = new THREE.LineLoop(new THREE.BufferGeometry(), matObstPoly);
rosRoot.add(meshObstPoly);


// ==========================================
// 2. WebSocket 与 UI 控制逻辑
// ==========================================
const ws = new WebSocket('ws://127.0.0.1:8080');
const statusDiv = document.getElementById('status');
const logOutput = document.getElementById('log-output');

// 维护各个数据流的开关状态 (默认 true)
const streamState = {
    point_cloud: true,
    agv_position: true,
    obst_polygon: true,
    model_polygon: true,
    scan2pointcloud: true,
    obst_pcl: true
};

ws.onopen = () => {
    statusDiv.textContent = '已连接到网关 (127.0.0.1:8080)';
    statusDiv.classList.add('connected');

    // 连接成功后，默认自动请求开启所有数据流
    Object.keys(streamState).forEach(key => {
        ws.send(`start_${key}`);
    });
};

ws.onclose = () => {
    statusDiv.textContent = '连接已断开';
    statusDiv.classList.remove('connected');
};

// 绑定所有的开关按钮
Object.keys(streamState).forEach(key => {
    const btn = document.getElementById(`btn-${key}`);
    // 初始化按钮样式 (红色 Stop)
    btn.classList.add('stop-btn');

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

// ==========================================
// 3. WebSocket 数据解析与渲染
// ==========================================
ws.onmessage = (event) => {
    try {
        const msg = JSON.parse(event.data);
        const type = msg.type;
        const res = msg.payload;

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
                agvGroup.position.set(data.x, data.y, data.z);
                agvGroup.rotation.z = data.theta; // 偏航角
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