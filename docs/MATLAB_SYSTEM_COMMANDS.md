# MATLAB 시스템 명령 실행 - MathWorks 표준

## 공식 문서
- https://kr.mathworks.com/help/matlab/matlab_external/run-external-commands-scripts-and-programs.html
- https://kr.mathworks.com/help/matlab/matlab_external/changing-environment-variables-for-shell-escape-functions.html

## MATLAB에서 CLI 명령 실행 방법

### 1. 환경 변수 설정 (MATLAB 표준)
```matlab
% 환경 변수 설정 - setenv() 함수 사용
setenv('ROS_DOMAIN_ID', '0');
setenv('GZ_VERSION', 'harmonic');
setenv('DISPLAY', ':0');

% 환경 변수 확인 - getenv() 함수 사용
current_domain_id = getenv('ROS_DOMAIN_ID');
current_path = getenv('PATH');
```

**중요**: MATLAB의 `setenv()`로 설정한 환경 변수는 **자식 프로세스(system 명령으로 실행되는 프로세스)에 상속됩니다**.

### 2. 시스템 명령 실행
```matlab
% 옵션 1: system() 함수 (권장 - 결과와 상태 코드 반환)
[status, result] = system('command here');
if status == 0
    fprintf('Success: %s\n', result);
else
    fprintf('Failed with status %d\n', status);
end

% 옵션 2: ! (bang) 연산자 (간단한 명령어용)
!echo "Hello from bash"
```

### 3. 명령어 연결 (UNIX/Linux)
```matlab
% UNIX/Linux에서는 세미콜론(;)으로 명령어 연결
cmd = 'source /opt/ros/humble/setup.bash; ros2 topic list';
[status, result] = system(cmd);
```

### 4. 백그라운드 실행 및 PID 캡처
```matlab
% 1단계: 환경 변수 미리 설정
setenv('ROS_DOMAIN_ID', '0');
setenv('GZ_VERSION', 'harmonic');

% 2단계: 명령어 문자열 구성
cmd = sprintf('source ~/.bashrc; my_launch_script > /tmp/log.txt 2>&1 & echo $! > /tmp/pid.txt');

% 3단계: system() 함수로 실행
[status, ~] = system(cmd);

% 4단계: PID 파일 읽기
if exist('/tmp/pid.txt', 'file')
    pid_str = strtrim(fileread('/tmp/pid.txt'));
    pid = str2double(pid_str);
    fprintf('Process PID: %d\n', pid);
end
```

## 우리 코드에 적용된 사항

### integrated_main.m
```matlab
% 환경 변수 먼저 설정 (MATLAB 표준)
setenv('ROS_DOMAIN_ID', '0');
setenv('GZ_VERSION', 'harmonic');
setenv('DISPLAY', ':0');

% 그 후에 매니저 생성 및 시뮬레이션 시작
sim = SimulationManager('verbose', true);
sim = sim.start_simulation('auto_launch', true);
```

### SimulationManager.m
```matlab
% 1. 환경 변수 설정
setenv('ROS_DOMAIN_ID', '0');
setenv('GZ_VERSION', 'harmonic');

% 2. 명령어 구성 (세미콜론으로 연결)
launch_cmd = sprintf([...
    'source /opt/ros/humble/setup.bash; ' ...
    'if [ -f %s/install/setup.bash ]; then source %s/install/setup.bash; fi; ' ...
    'bash %s'], ardu_ws, ardu_ws, launch_script);

% 3. 백그라운드 실행
bg_cmd = sprintf('nohup bash -c ''%s'' > %s 2>&1 & echo $! > %s', ...
    launch_cmd, log_file, pid_file);
[status, result] = system(bg_cmd);

% 4. PID 확인
if exist(pid_file, 'file')
    pid_str = strtrim(fileread(pid_file));
    obj.launch_pid = str2double(pid_str);
end
```

## 핵심 개선사항

1. **환경 변수 관리**: `setenv()` / `getenv()` 사용 (MATLAB 표준)
2. **명령어 연결**: `;` (세미콜론) 사용 (UNIX 표준)
3. **상태 확인**: `system()` 함수의 반환값 검사
4. **PID 추적**: 파일 기반으로 프로세스 추적
5. **오류 처리**: 명확한 상태 메시지와 로그 파일 확인

## 테스트 방법

```matlab
% 1. 환경 변수 설정 테스트
setenv('TEST_VAR', 'hello');
[~, result] = system('echo $TEST_VAR');
fprintf('Environment: %s\n', result);

% 2. 명령어 실행 테스트
[status, result] = system('ros2 topic list');
fprintf('Topic list (status=%d):\n%s\n', status, result);

% 3. 백그라운드 프로세스 테스트
[status, ~] = system('sleep 10 & echo $!');
```

## 참고사항

- MATLAB에서 `setenv()`로 설정한 변수는 현재 MATLAB 세션 내에서만 유효
- `system()` 함수는 각 호출마다 새로운 쉘 인스턴스 생성 → 변수가 상속됨
- 백그라운드 프로세스(`&`)는 MATLAB 프로세스 종료 후에도 계속 실행됨
- 로그 파일(`nohup.out` 또는 지정된 파일)로 출력 확인 가능
