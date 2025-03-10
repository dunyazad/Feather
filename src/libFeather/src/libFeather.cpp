#include <libFeather.h>

libFeather::libFeather()
{
}

libFeather::~libFeather()
{
}

void libFeather::Initialize()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
}

void libFeather::Terminate()
{
    ImPlot::DestroyContext();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    for (auto& s : shaders)
    {
        if (nullptr != s)
        {
            delete s;
        }
    }
    shaders.clear();

    for (auto& w : featherWindows)
    {
        if (nullptr != w)
        {
            delete w;
        }
    }
    featherWindows.clear();
}

void libFeather::Run()
{
    ImGui_ImplGlfw_InitForOpenGL(featherWindows.front()->GetGLFWwindow(), true);
    ImGui_ImplOpenGL3_Init("#version 330");

    while (!glfwWindowShouldClose(glfwGetCurrentContext()))
    {
        glfwPollEvents();

        if (needFontReload) {
            ReloadFont(fontSize);
            needFontReload = false;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        //ShowUIPanel();
        //ShowGraphPanel();
        //ShowTeapotPanel();

        ImGui::Render();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(glfwGetCurrentContext());
    }
}

FeatherWindow* libFeather::CreateWindow(ui32 width, ui32 height)
{
    auto w = new FeatherWindow();
    w->Initialize(width, height);
    featherWindows.push_back(w);
    return w;
}

Shader* libFeather::CreateShader()
{
    auto s = new Shader();
    s->Initialize();
    shaders.push_back(s);
    return s;
}

void libFeather::ReloadFont(float newFontSize)
{
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->Clear();
    io.Fonts->AddFontFromFileTTF("../../res/Fonts/NanumGothic/NanumGothic-Regular.ttf", newFontSize);
    ImGui_ImplOpenGL3_DestroyFontsTexture();
    ImGui_ImplOpenGL3_CreateFontsTexture();
}

// Window dimensions
const int WIDTH = 800, HEIGHT = 600;

float view[16];
float projection[16];

// UI 관련 변수
float amplitude = 1.0f;
float frequency = 1.0f;
float fontSize = 18.0f;
float rotationAngle = 0.0f;
bool needFontReload = false;  // 폰트 리로드 여부

GLuint shaderProgram, VAO, VBO;

// Vertex Shader (변환 행렬 적용)
const char* vertexShaderSource = R"(
    // Vertex Shader (변환 행렬 및 카메라 설정)
    #version 330 core

    layout (location = 0) in vec3 aPos;  // 입점 위치 (Vertex Position)

    uniform mat4 transform;  // 회전 변환 행렬
    uniform mat4 projection; // 투영 행렬
    uniform mat4 view;       // 카메라 뷰 행렬

    void main() {
        // 변환 행렬을 통해 위치 변환
        gl_Position = projection * view * transform * vec4(aPos, 1.0);
    }
)";

// Fragment Shader
const char* fragmentShaderSource = R"(
    // Fragment Shader (색상 설정)
    #version 330 core

    out vec4 FragColor;

    void main() {
        // 단순한 색상 설정 (붉은색)
        FragColor = vec4(0.8, 0.3, 0.2, 1.0); // RGB 값 (붉은색)
    }
)";

// 셰이더 컴파일 및 링크 오류 체크 함수
void CheckShaderCompileErrors(GLuint shader, const std::string& type) {
    GLint success;
    GLchar infoLog[1024];
    if (type == "VERTEX" || type == "FRAGMENT") {
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(shader, 1024, NULL, infoLog);
            std::cerr << type << " Shader compilation failed: " << infoLog << std::endl;
        }
    }
    else if (type == "PROGRAM") {
        glGetProgramiv(shader, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(shader, 1024, NULL, infoLog);
            std::cerr << "Shader Program linking failed: " << infoLog << std::endl;
        }
    }
}

// OpenGL 초기화
void InitOpenGL() {
    glEnable(GL_DEPTH_TEST);  // 깊이 테스트 활성화
    glViewport(0, 0, WIDTH, HEIGHT);  // Viewport 설정

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);

    // 삼각형(Teapot 대용)
    float vertices[] = {
        -0.5f, -0.5f,  0.0f,
         0.5f, -0.5f,  0.0f,
         0.0f,  0.5f,  0.0f
    };

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

// ImGui 초기화
void InitImGui(GLFWwindow* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

// ImGui 정리
void CleanupImGui() {
    ImPlot::DestroyContext();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

// 폰트 리로드
void ReloadFont(float newFontSize) {
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->Clear();
    io.Fonts->AddFontFromFileTTF("../../res/Fonts/NanumGothic/NanumGothic-Regular.ttf", newFontSize);
    ImGui_ImplOpenGL3_DestroyFontsTexture();
    ImGui_ImplOpenGL3_CreateFontsTexture();
}

// Projection 행렬 추가
void SetProjectionMatrix(GLuint shaderProgram, int width, int height) {
    float fov = 45.0f;  // Field of view
    float near = 0.1f;  // Near plane
    float far = 100.0f; // Far plane
    float aspect = (float)width / (float)height;

    float projection[16];
    float tanHalfFov = tanf(fov / 2.0f * M_PI / 180.0f);

    memset(projection, 0, sizeof(projection));

    projection[0] = 1.0f / (aspect * tanHalfFov);
    projection[5] = 1.0f / tanHalfFov;
    projection[10] = -(far + near) / (far - near);
    projection[11] = -1.0f;
    projection[14] = -(2.0f * far * near) / (far - near);
    projection[15] = 0.0f;

    GLuint projLoc = glGetUniformLocation(shaderProgram, "projection");
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, projection);
}

// View 행렬 설정 (카메라)
void SetViewMatrix(GLuint shaderProgram) {
    float cameraPos[3] = { 0.0f, 0.0f, 3.0f };
    float cameraTarget[3] = { 0.0f, 0.0f, 0.0f };
    float up[3] = { 0.0f, 1.0f, 0.0f };

    float forward[3] = { cameraPos[0] - cameraTarget[0], cameraPos[1] - cameraTarget[1], cameraPos[2] - cameraTarget[2] };
    float right[3] = { up[1] * forward[2] - up[2] * forward[1], up[2] * forward[0] - up[0] * forward[2], up[0] * forward[1] - up[1] * forward[0] };
    float newUp[3] = { forward[1] * right[2] - forward[2] * right[1], forward[2] * right[0] - forward[0] * right[2], forward[0] * right[1] - forward[1] * right[0] };

    float view[16] = { 0 };
    view[0] = right[0];
    view[1] = right[1];
    view[2] = right[2];
    view[4] = newUp[0];
    view[5] = newUp[1];
    view[6] = newUp[2];
    view[8] = -forward[0];
    view[9] = -forward[1];
    view[10] = -forward[2];
    view[15] = 1.0f;

    view[12] = -(cameraPos[0] * view[0] + cameraPos[1] * view[4] + cameraPos[2] * view[8]);
    view[13] = -(cameraPos[0] * view[1] + cameraPos[1] * view[5] + cameraPos[2] * view[9]);
    view[14] = -(cameraPos[0] * view[2] + cameraPos[1] * view[6] + cameraPos[2] * view[10]);

    GLuint viewLoc = glGetUniformLocation(shaderProgram, "view");
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, view);
}

// 회전 행렬 계산
void SetTransformMatrix(GLuint shaderProgram) {
    float transform[16] = { 0 };

    float radians = rotationAngle * M_PI / 180.0f;
    float cosA = cosf(radians);
    float sinA = sinf(radians);

    transform[0] = cosA;
    transform[1] = sinA;
    transform[4] = -sinA;
    transform[5] = cosA;
    transform[10] = 1.0f;
    transform[12] = 0.0f;
    transform[13] = 0.0f;
    transform[14] = -2.0f;
    transform[15] = 1.0f;

    GLuint transformLoc = glGetUniformLocation(shaderProgram, "transform");
    glUniformMatrix4fv(transformLoc, 1, GL_FALSE, transform);
}

// Teapot 렌더링 (회전 변환 적용)
void RenderTeapot() {
    glUseProgram(shaderProgram);
    glBindVertexArray(VAO);

    SetProjectionMatrix(shaderProgram, WIDTH, HEIGHT);
    SetViewMatrix(shaderProgram);
    SetTransformMatrix(shaderProgram);

    glDrawArrays(GL_TRIANGLES, 0, 3);
}

float angle = 0.0f; // 삼각형의 초기 회전 각도
float scale = 50.0f; // 삼각형 크기 조정

// OpenGL에서 삼각형을 그리는 함수
void RenderTriangle(ImDrawList* draw_list, const ImVec2& offset, float angle, float scale) {
    // 원본 삼각형 정점 (크기를 늘리기 전에 원래의 크기로 설정)
    float vertices[] = {
        0.0f,  0.5f, 0.0f,  // Top vertex
       -0.5f, -0.5f, 0.0f,  // Bottom left vertex
        0.5f, -0.5f, 0.0f   // Bottom right vertex
    };

    // 회전 행렬 계산
    float cosAngle = cosf(angle);
    float sinAngle = sinf(angle);

    float rotationMatrix[9] = {
        cosAngle, -sinAngle, 0.0f,
        sinAngle,  cosAngle, 0.0f,
        0.0f,      0.0f,     1.0f
    };

    // 삼각형의 각 버텍스를 회전 행렬로 변환하고 크기 조정
    for (int i = 0; i < 3; ++i) {
        float x = vertices[i * 3];
        float y = vertices[i * 3 + 1];

        // 회전 행렬 적용
        vertices[i * 3] = rotationMatrix[0] * x + rotationMatrix[1] * y;
        vertices[i * 3 + 1] = rotationMatrix[3] * x + rotationMatrix[4] * y;

        // 크기 조정
        vertices[i * 3] *= scale;
        vertices[i * 3 + 1] *= scale;
    }

    // 변환된 좌표로 삼각형을 그리기
    for (int i = 0; i < 3; ++i) {
        vertices[i * 3] += offset.x;
        vertices[i * 3 + 1] += offset.y;
    }

    // 삼각형 그리기
    draw_list->AddTriangle(
        ImVec2(vertices[0], vertices[1]),
        ImVec2(vertices[3], vertices[4]),
        ImVec2(vertices[6], vertices[7]),
        IM_COL32(255, 0, 0, 255),  // Red color
        2.0f  // Thickness
    );
}


// OpenGL Teapot 렌더링
void ShowTeapotPanel() {
    ImGui::Begin("Teapot Renderer");

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ImGui::SliderFloat("Angle", &angle, 0.0f, 2.0f * 3.14159f);
    ImGui::SliderFloat("Scale", &scale, 0.1f, 2000.0f);  // 크기 조정 슬라이더

    //RenderTeapot();
     // 패널 안에 삼각형 그리기
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 panel_offset = ImGui::GetCursorScreenPos();  // 패널의 좌표
    RenderTriangle(draw_list, panel_offset, angle, scale);

    ImGui::End();
}

// 그래프 출력
void ShowGraphPanel() {
    ImGui::Begin("Sine Wave Plot");

    std::vector<float> x_data(100);
    std::vector<float> y_data(100);
    for (size_t i = 0; i < x_data.size(); i++) {
        x_data[i] = i * 0.1f;
        y_data[i] = amplitude * sin(frequency * x_data[i]);
    }

    if (ImPlot::BeginPlot("Sine Wave")) {
        ImPlot::PlotLine("sin(x)", x_data.data(), y_data.data(), x_data.size());
        ImPlot::EndPlot();
    }

    ImGui::End();
}

// UI 패널
void ShowUIPanel() {
    ImGui::Begin("Control Panel");

    if (ImGui::SliderFloat("Font Size", &fontSize, 12.0f, 32.0f)) {
        needFontReload = true;
    }

    if (ImGui::Button("Reset")) {
        amplitude = 1.0f;
        frequency = 1.0f;
        fontSize = 18.0f;
        needFontReload = true;
    }

    ImGui::End();
}

// 렌더 루프
void RenderLoop() {
    while (!glfwWindowShouldClose(glfwGetCurrentContext())) {
        glfwPollEvents();
        rotationAngle += 1.0f;

        if (needFontReload) {
            ReloadFont(fontSize);
            needFontReload = false;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ShowUIPanel();
        ShowGraphPanel();
        ShowTeapotPanel();

        ImGui::Render();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(glfwGetCurrentContext());
    }
}

//
//#include <glad/glad.h>
//#include <GLFW/glfw3.h>
//#include <imgui.h>
//#include <backends/imgui_impl_glfw.h>
//#include <backends/imgui_impl_opengl3.h>
//#include <cmath>
//
//float angle = 0.0f; // 삼각형의 초기 회전 각도
//float scale = 50.0f; // 삼각형 크기 조정
//
//// OpenGL에서 삼각형을 그리는 함수
//void RenderTriangle(ImDrawList* draw_list, const ImVec2& offset, float angle, float scale) {
//    // 원본 삼각형 정점 (크기를 늘리기 전에 원래의 크기로 설정)
//    float vertices[] = {
//        0.0f,  0.5f, 0.0f,  // Top vertex
//       -0.5f, -0.5f, 0.0f,  // Bottom left vertex
//        0.5f, -0.5f, 0.0f   // Bottom right vertex
//    };
//
//    // 회전 행렬 계산
//    float cosAngle = cosf(angle);
//    float sinAngle = sinf(angle);
//
//    float rotationMatrix[9] = {
//        cosAngle, -sinAngle, 0.0f,
//        sinAngle,  cosAngle, 0.0f,
//        0.0f,      0.0f,     1.0f
//    };
//
//    // 삼각형의 각 버텍스를 회전 행렬로 변환하고 크기 조정
//    for (int i = 0; i < 3; ++i) {
//        float x = vertices[i * 3];
//        float y = vertices[i * 3 + 1];
//
//        // 회전 행렬 적용
//        vertices[i * 3] = rotationMatrix[0] * x + rotationMatrix[1] * y;
//        vertices[i * 3 + 1] = rotationMatrix[3] * x + rotationMatrix[4] * y;
//
//        // 크기 조정
//        vertices[i * 3] *= scale;
//        vertices[i * 3 + 1] *= scale;
//    }
//
//    // 변환된 좌표로 삼각형을 그리기
//    for (int i = 0; i < 3; ++i) {
//        vertices[i * 3] += offset.x;
//        vertices[i * 3 + 1] += offset.y;
//    }
//
//    // 삼각형 그리기
//    draw_list->AddTriangle(
//        ImVec2(vertices[0], vertices[1]),
//        ImVec2(vertices[3], vertices[4]),
//        ImVec2(vertices[6], vertices[7]),
//        IM_COL32(255, 0, 0, 255),  // Red color
//        2.0f  // Thickness
//    );
//}
//
//
//void libFeather::Test() {
//    if (!glfwInit()) {
//        std::cerr << "GLFW initialization failed!" << std::endl;
//        return;
//    }
//
//    // GLFW 윈도우 생성
//    GLFWwindow* window = glfwCreateWindow(1280, 720, "Rotating Triangle in ImGui Panel", nullptr, nullptr);
//    if (!window) {
//        glfwTerminate();
//        return;
//    }
//
//    glfwMakeContextCurrent(window);
//    glfwSwapInterval(1); // V-Sync
//    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
//
//    // ImGui 초기화
//    IMGUI_CHECKVERSION();
//    ImGui::CreateContext();
//    ImGuiIO& io = ImGui::GetIO();
//    ImGui::StyleColorsDark();
//    ImGui_ImplGlfw_InitForOpenGL(window, true);
//    ImGui_ImplOpenGL3_Init("#version 130");
//
//    while (!glfwWindowShouldClose(window)) {
//        glfwPollEvents();
//
//        // ImGui Frame 시작
//        ImGui_ImplOpenGL3_NewFrame();
//        ImGui_ImplGlfw_NewFrame();
//        ImGui::NewFrame();
//
//        // OpenGL 렌더링
//        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
//        glClear(GL_COLOR_BUFFER_BIT);
//
//
//        // ImGui로 Panel 만들기
//        ImGui::Begin("Control Panel");
//        ImGui::Text("Rotate Triangle in Panel");
//        ImGui::SliderFloat("Angle", &angle, 0.0f, 2.0f * 3.14159f);
//        ImGui::SliderFloat("Scale", &scale, 0.1f, 2000.0f);  // 크기 조정 슬라이더
//        ImGui::End();
//
//        // ImGui 창을 그릴 공간을 설정
//        ImGui::Begin("Triangle Panel");
//
//        // 패널 안에 삼각형 그리기
//        ImDrawList* draw_list = ImGui::GetWindowDrawList();
//        ImVec2 panel_offset = ImGui::GetCursorScreenPos();  // 패널의 좌표
//        RenderTriangle(draw_list, panel_offset, angle, scale);
//
//        ImGui::End();
//
//        // ImGui 렌더링
//        ImGui::Render();
//        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
//
//        // 버퍼 교체
//        glfwSwapBuffers(window);
//    }
//
//    // 정리
//    ImGui_ImplOpenGL3_Shutdown();
//    ImGui_ImplGlfw_Shutdown();
//    ImGui::DestroyContext();
//
//    glfwDestroyWindow(window);
//    glfwTerminate();
//}
