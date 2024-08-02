#ifdef WITH_OPENGL

#include "GUIRendererGL.hpp"
#include <iostream>

int windowWidth = 1280; // Default width
int windowHeight = 960; // Default height


const char* vertexShaderSource = R"glsl(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec2 aTexCoords;

    out vec2 TexCoords;

    void main() {
        gl_Position = vec4(aPos, 1.0);
        TexCoords = aTexCoords;
    }

)glsl";

const char* fragmentShaderSource = R"glsl(
   #version 330 core
    in vec2 TexCoords;
    out vec4 FragColor;
    uniform sampler2D texture1;  // Original frame
    uniform sampler2D texture2;  // Processed frame

    void main() {
        if (TexCoords.x < 0.5) {
            FragColor = texture(texture1, vec2(TexCoords.x * 2.0, TexCoords.y));
        } else {
            FragColor = texture(texture2, vec2((TexCoords.x - 0.5) * 2.0, TexCoords.y));
        }
    }



)glsl";


namespace vision_core{


GUIRendererGL::GUIRendererGL() : GUIRenderer() {
}


void GUIRendererGL::start(Configs config) {
    initOpenGL();
    createTextures();
    this->config = config;
    if (config.useRenderingThread) {
    renderThread = std::thread(&GUIRendererGL::renderLoop, this);
    std::cout << "Rendering with OpenGL" << std::endl;

    }
}


void GUIRendererGL::renderLoop() {
    glfwMakeContextCurrent(window);  // Make the context current on this thread
    glViewport(0, 0, windowWidth, windowHeight);

    while (!stopRequested) {
        std::unique_lock<std::mutex> lock(frameMutex);
        frameCondVar.wait(lock, [this] { return newFrameAvailable || stopRequested; });
        if (stopRequested) break;

        if (window) {
            if (glfwWindowShouldClose(window)) {
                stopRequested = true;
                break;
            }
            display();
            glfwSwapBuffers(window);
            glfwPollEvents();
        } 
        newFrameAvailable = false;
    }

        glfwMakeContextCurrent(NULL);  // Detach the context when exiting the thread
}

void GUIRendererGL::display() {
    if (!window) return;

    glUseProgram(shaderProgram);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);  // Set clear color to black
    glClear(GL_COLOR_BUFFER_BIT);

    if (newFrameAvailable) {
        updateTexture(textureIDOriginal, originalFrame);
        updateTexture(textureIDProcessed, processedFrame);
        newFrameAvailable = false;
    }

    // Bind texture for the original frame
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, textureIDOriginal);
    glUniform1i(glGetUniformLocation(shaderProgram, "texture1"), 0);

    // Bind texture for the processed frame
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, textureIDProcessed);
    glUniform1i(glGetUniformLocation(shaderProgram, "texture2"), 1);

    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, 12, GL_UNSIGNED_INT, 0);  // Draw both quads
    glBindVertexArray(0);

    glfwSwapBuffers(window);

    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR) {
        std::cerr << "OpenGL error: " << std::hex << err << std::endl;
    }
}




bool GUIRendererGL::isStopRequested() const {
  return stopRequested;
}

GLuint GUIRendererGL::compileShader(GLenum type, const GLchar* source) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint success;
    GLchar infoLog[512];
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(shader, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER::COMPILATION_FAILED\n" << infoLog << std::endl;
    }

    return shader;
}

GLuint GUIRendererGL::createShaderProgram(const char* vShaderCode, const char* fShaderCode) {
    GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vShaderCode);
    GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fShaderCode);
    
    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    GLint success;
    GLchar infoLog[512];
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        std::cerr << "ERROR::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return shaderProgram;
}


void GUIRendererGL::setupVertexDataAndBuffers() {
    
    // Define vertex positions and texture coordinates
    float vertices[] = {
        // positions        // texture coords
        -1.0f,  1.0f, 0.0f,  0.0f, 0.0f, // Top Left
        1.0f,  1.0f, 0.0f,   1.0f, 0.0f, // Top Right
        1.0f, -1.0f, 0.0f,   1.0f, 1.0f, // Bottom Right
        -1.0f, -1.0f, 0.0f,  0.0f, 1.0f  // Bottom Left
    };


    unsigned int indices[] = {
        0, 1, 2,   // First Triangle
        2, 3, 0    // Second Triangle
    };


    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // Texture coordinate attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);  // Unbind VAO
}


void GUIRendererGL::initOpenGL() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW." << std::endl;
        exit(-1);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(800, 600, "YOLO Detector", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window." << std::endl;
        glfwTerminate();
        exit(-1);
    }

    glfwMakeContextCurrent(window);
    
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        exit(-1);
    }

    // Create Shader Program
    shaderProgram = createShaderProgram(vertexShaderSource, fragmentShaderSource);

    // Setup vertex data and buffers and configure vertex attributes
    setupVertexDataAndBuffers();
}



void GUIRendererGL::closeOpenGL() {
    if (window) {
    glfwDestroyWindow(window);
    window = nullptr;
    }
    glfwTerminate();
}


void GUIRendererGL::createTextures() {
    glGenTextures(1, &textureIDOriginal);
    glGenTextures(1, &textureIDProcessed);
    glBindTexture(GL_TEXTURE_2D, textureIDOriginal);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindTexture(GL_TEXTURE_2D, textureIDProcessed);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

void GUIRendererGL::updateTexture(GLuint textureId, const cv::Mat& image) {
    glBindTexture(GL_TEXTURE_2D, textureId);
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
        std::cerr << "OpenGL error after glBindTexture: " << std::hex << error << std::endl;
        return;
    }

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, image.data);
    error = glGetError();
    if (error != GL_NO_ERROR) {
        std::cerr << "OpenGL error after glTexImage2D: " << std::hex << error << std::endl;
        return;
    }

    glGenerateMipmap(GL_TEXTURE_2D);
    error = glGetError();
    if (error != GL_NO_ERROR) {
        std::cerr << "OpenGL error after glGenerateMipmap: " << std::hex << error << std::endl;
    }
}


} // namespace vision_core

#endif // WITH_OPENGL