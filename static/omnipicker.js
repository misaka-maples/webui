function refreshSerialPorts() {
    fetch('/serial/ports')
        .then(res => res.json())
        .then(data => {
            const select = document.getElementById("serialSelect");
            select.innerHTML = ""; // 清空旧选项
            data.ports.forEach(port => {
                const option = document.createElement("option");
                option.value = port;
                option.textContent = port;
                select.appendChild(option);
            });
        })
        .catch(err => alert("刷新串口失败: " + err));
}

function openSerialPort() {
    const port = document.getElementById("serialSelect").value;
    fetch('/serial/open', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ port: port })
    })
        .then(res => res.json())
        .then(data => alert(data.message))
        .catch(err => alert("打开串口失败: " + err));
}

function reconnectSerial() {
    fetch('/serial/reconnect', { method: 'POST' })
        .then(res => res.json())
        .then(data => alert(data.message))
        .catch(err => alert("重连失败: " + err));
}

function updateGripper(gripperId) {
    const slider = document.getElementById(`gripper${gripperId}Slider`);
    const value = slider.value;
    console.log(`夹爪 ${gripperId} 实时设置为 ${value} 度`);

    // 发送到后端（可选加防抖 debounce）
    fetch(`/gripper/${gripperId}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ value: value })
    });
}
function startGripperThread() {
    fetch('/gripper/start', { method: 'POST' })
        .then(res => res.json())
        .then(data => alert(data.message))
        .catch(err => alert("启动夹爪线程失败: " + err));
}
function stopGripperThread() {
    fetch('/gripper/stop', { method: 'POST' })
        .then(res => res.json())
        .then(data => alert(data.message))
        .catch(err => alert("停止夹爪线程失败: " + err));
}
