function togglePasswordVisibility() {
    const passwordInput = document.getElementById('password');
    const eyeButton = document.getElementById('togglePasswordBtn');
    
    // 切换输入框类型
    const isPassword = passwordInput.type === 'password';
    passwordInput.type = isPassword ? 'text' : 'password';
    
    // 切换眼睛图标状态
    eyeButton.classList.toggle('active', !isPassword);
}
document.getElementById('togglePasswordBtn').addEventListener('click', togglePasswordVisibility);

async function fetchSystemInfo() {
    try {
        const response = await fetch('/api/system-info');
        const data = await response.json();

        document.getElementById('systemVersion').textContent = data.system_version || '未知';
        document.getElementById('deviceName').textContent = data.device_name || '未知';
        
    } catch (error) {
        console.error('获取系统信息失败:', error);
    }
}

// 增强版登录提交（带凭证管理）
document.getElementById('loginForm').addEventListener('submit', async function (e) {
    e.preventDefault();
    const formData = {
        username: document.getElementById('username').value.trim(),
        password: document.getElementById('password').value.trim(),
        errors: {
            username: document.getElementById('usernameError'),
            password: document.getElementById('passwordError'),
            login: document.getElementById('loginError')
        }
    };

    // 重置错误状态
    Object.values(formData.errors).forEach(el => {
        el.textContent = '';
        el.classList.remove('show');
    });

    // 客户端验证
    let isValid = true;
    if (!formData.username) {
        formData.errors.username.textContent = '用户名不能为空';
        formData.errors.username.classList.add('show');
        isValid = false;
    } else if (!/^[a-zA-Z][a-zA-Z0-9_]{0,31}$/.test(formData.username)) {
        formData.errors.username.textContent = '用户名不符合规范（字母开头，4-32位字母数字下划线）';
        formData.errors.username.classList.add('show');
        isValid = false;
    }

    if (!formData.password) {
        formData.errors.password.textContent = '密码不能为空';
        formData.errors.password.classList.add('show');
        isValid = false;
    }

    if (!isValid) return;

    try {
        const response = await fetch('/api/login', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                username: formData.username,
                password: formData.password
            }),
            credentials: 'include' 
        });

        const data = await response.json();
        
        if (response.ok && data.success) {
            // 延迟跳转确保Cookie生效
            setTimeout(() => {
                window.location.href = data.redirect || '/';
            }, 300);
        } else {
            throw new Error(data.message || '未知登录错误');
        }
    } catch (error) {
        console.error('登录失败:', error);
        formData.errors.login.textContent = error.message.replace('pam_', '');
        formData.errors.login.classList.add('show');
    }
});

// 初始化系统信息
window.addEventListener('DOMContentLoaded', fetchSystemInfo);
