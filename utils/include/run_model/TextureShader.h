#pragma once

namespace pangolin {

const std::string shader = R"Shader(
/////////////////////////////////////////
@start vertex
#version 120


    uniform mat4 KT_cw;
    attribute vec3 vertex;
    attribute vec2 uv;
    varying vec2 vUV;
    void main() {
        vUV = uv;
        gl_Position = KT_cw * vec4(vertex, 1.0);
    }

/////////////////////////////////////////
@start fragment
#version 120
    varying vec2 vUV;
    uniform sampler2D texture;

void main() {
    gl_FragColor = texture2D(texture, vUV);
}
)Shader";
}
