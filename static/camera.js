
function postAction(action) {
    const input = document.getElementById("inputBox").value;
fetch(`/action/${action}`, {
    method: 'POST',
headers: {'Content-Type': 'application/json' },
body: JSON.stringify({input: input })
    })
        .then(response => response.json())
        .then(data => {
    alert(data.message);

if (action === 'start_camera') {
    // 刷新所有摄像头视频 <img> 的 src
    reloadVideoFeeds();
            }

if (action === 'stop_camera') {
    // 清空视频 <img>，让它们停止加载
    clearVideoFeeds();
            }
        })
        .catch(error => alert("Error: " + error));
}

function reloadVideoFeeds() {
    const cams = ['top', 'left_wrist', 'right_wrist'];
    cams.forEach(name => {
        const img = document.getElementById(`cam-${name.replace('_', '-')}`);
if (img) {
    // 强制刷新 MJPEG 流：加时间戳防止浏览器缓存
    img.src = `/video_feed/${name}?ts=` + Date.now();
        }
    });
}

function clearVideoFeeds() {
    const cams = ['top', 'left_wrist', 'right_wrist'];
    cams.forEach(name => {
        const img = document.getElementById(`cam-${name.replace('_', '-')}`);
if (img) {
    img.src = "";
        }
    });
}
function submitNote() {
    const note = document.getElementById("anotherInputBox").value;

    fetch('/submit_note', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ note: note })
    })
        .then(response => response.json())
        .then(data => {
            alert('备注提交成功: ' + data.message);
        })
        .catch(error => {
            alert('提交失败');
            console.error('Error:', error);
        });
}

function startTask() {postAction("start_task"); }
function stopTask() {postAction("stop_task"); }
function startCamera() {postAction("start_camera"); }
function stopCamera() {postAction("stop_camera"); }
