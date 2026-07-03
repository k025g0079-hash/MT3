#include <Novice.h>
#include <math.h>
#include <assert.h>
#include <utility> 
#include <algorithm> // std::min, std::max用
#define ImGui_ImplDX12_RGB_Color
#include <imgui.h>

// --- 構造体定義 ---
struct Vector {
	float x;
	float y;
	float z;
};

struct Matrix4x4 {
	float m[4][4];
};

// 2次ベジェ曲線構造体 (制御点0, 1, 2)
struct BezierCurve {
	Vector p0; // controlPoints[0]
	Vector p1; // controlPoints[1]
	Vector p2; // controlPoints[2]
};

// --- ベクトル演算 ---
Vector Add(Vector v1, Vector v2) { return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z }; }
Vector Subtract(Vector v1, Vector v2) { return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z }; }
Vector Multiply(float k, Vector v) { return { k * v.x, k * v.y, k * v.z }; }
float Dot(Vector v1, Vector v2) { return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z; }
float LengthSq(Vector v) { return v.x * v.x + v.y * v.y + v.z * v.z; }
float Length(Vector v) { return sqrtf(LengthSq(v)); }

// クランプ関数
float Clamp(float value, float min, float max) {
	if (value < min) return min;
	if (value > max) return max;
	return value;
}

// --- 行列演算 ---
Matrix4x4 Multiply(Matrix4x4 A, Matrix4x4 B) {
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			result.m[i][j] = 0.0f;
			for (int k = 0; k < 4; k++) {
				result.m[i][j] += A.m[i][k] * B.m[k][j];
			}
		}
	}
	return result;
}

Matrix4x4 Inverse(Matrix4x4 m) {
	Matrix4x4 result{};
	float a[4][8] = {};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			a[i][j] = m.m[i][j];
			a[i][j + 4] = (i == j) ? 1.0f : 0.0f;
		}
	}
	for (int i = 0; i < 4; i++) {
		int pivotRow = i;
		float maxVal = fabsf(a[i][i]);
		for (int k = i + 1; k < 4; k++) {
			if (fabsf(a[k][i]) > maxVal) {
				maxVal = fabsf(a[k][i]);
				pivotRow = k;
			}
		}
		if (pivotRow != i) {
			for (int j = 0; j < 8; j++) std::swap(a[i][j], a[pivotRow][j]);
		}
		float pivot = a[i][i];
		if (fabsf(pivot) < 1e-6f) pivot = 1e-6f;
		for (int j = 0; j < 8; j++) a[i][j] /= pivot;
		for (int k = 0; k < 4; k++) {
			if (k == i) continue;
			float factor = a[k][i];
			for (int j = 0; j < 8; j++) a[k][j] -= factor * a[i][j];
		}
	}
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) result.m[i][j] = a[i][j + 4];
	}
	return result;
}

Matrix4x4 MakeIdentity() {
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++) result.m[i][i] = 1.0f;
	return result;
}

Matrix4x4 MakeAffineMatrix(Vector scale, Vector rotate, Vector translate) {
	Matrix4x4 rotateX = { {
		{1.0f, 0.0f, 0.0f, 0.0f},
		{0.0f, cosf(rotate.x), sinf(rotate.x), 0.0f},
		{0.0f, -sinf(rotate.x), cosf(rotate.x), 0.0f},
		{0.0f, 0.0f, 0.0f, 1.0f}
	} };
	Matrix4x4 rotateY = { {
		{cosf(rotate.y), 0.0f, -sinf(rotate.y), 0.0f},
		{0.0f, 1.0f, 0.0f, 0.0f},
		{sinf(rotate.y), 0.0f, cosf(rotate.y), 0.0f},
		{0.0f, 0.0f, 0.0f, 1.0f}
	} };
	Matrix4x4 rotateZ = { {
		{cosf(rotate.z), sinf(rotate.z), 0.0f, 0.0f},
		{-sinf(rotate.z), cosf(rotate.z), 0.0f, 0.0f},
		{0.0f, 0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 0.0f, 1.0f}
	} };
	Matrix4x4 rotMat = Multiply(rotateX, Multiply(rotateY, rotateZ));

	Matrix4x4 result = { {
		{scale.x * rotMat.m[0][0], scale.x * rotMat.m[0][1], scale.x * rotMat.m[0][2], 0.0f},
		{scale.y * rotMat.m[1][0], scale.y * rotMat.m[1][1], scale.y * rotMat.m[1][2], 0.0f},
		{scale.z * rotMat.m[2][0], scale.z * rotMat.m[2][1], scale.z * rotMat.m[2][2], 0.0f},
		{translate.x, translate.y, translate.z, 1.0f}
	} };
	return result;
}

Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspect, float nearClip, float farClip) {
	Matrix4x4 result{};
	float cot = 1.0f / tanf(fovY / 2.0f);
	result.m[0][0] = cot / aspect;
	result.m[1][1] = cot;
	result.m[2][2] = farClip / (farClip - nearClip);
	result.m[2][3] = 1.0f;
	result.m[3][2] = (-nearClip * farClip) / (farClip - nearClip);
	return result;
}

Matrix4x4 MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth) {
	Matrix4x4 result{};
	result.m[0][0] = width / 2.0f;
	result.m[1][1] = -height / 2.0f;
	result.m[2][2] = maxDepth - minDepth;
	result.m[3][0] = left + width / 2.0f;
	result.m[3][1] = top + height / 2.0f;
	result.m[3][2] = minDepth;
	result.m[3][3] = 1.0f;
	return result;
}

// --- 座標変換 ---
Vector Transform(Vector vector, Matrix4x4 matrix) {
	Vector result{};
	float w = vector.x * matrix.m[0][3] + vector.y * matrix.m[1][3] + vector.z * matrix.m[2][3] + matrix.m[3][3];
	if (w == 0.0f) w = 1.0f;

	result.x = (vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] + matrix.m[3][0]) / w;
	result.y = (vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] + matrix.m[3][1]) / w;
	result.z = (vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] + matrix.m[3][2]) / w;
	return result;
}

Vector Transform3DTo2D(Vector position, Matrix4x4 worldMatrix, Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix, Matrix4x4 viewportMatrix) {
	Matrix4x4 wvpVpMatrix = Multiply(worldMatrix, Multiply(viewMatrix, Multiply(projectionMatrix, viewportMatrix)));
	return Transform(position, wvpVpMatrix);
}

// --- 衝突判定アルゴリズム ---

float ClosestDistanceSegmentToSegmentSq(Vector p1, Vector q1, Vector p2, Vector q2) {
	Vector d1 = Subtract(q1, p1);
	Vector d2 = Subtract(q2, p2);
	Vector r = Subtract(p1, p2);
	float a = Dot(d1, d1);
	float e = Dot(d2, d2);
	float f = Dot(d2, r);

	float s = 0.0f;
	float t = 0.0f;

	if (a <= 1e-6f && e <= 1e-6f) {
		s = 0.0f; t = 0.0f;
	}
	else if (a <= 1e-6f) {
		s = 0.0f;
		t = Clamp(f / e, 0.0f, 1.0f);
	}
	else {
		float c = Dot(d1, r);
		if (e <= 1e-6f) {
			t = 0.0f;
			s = Clamp(-c / a, 0.0f, 1.0f);
		}
		else {
			float b = Dot(d1, d2);
			float denom = a * e - b * b;

			if (fabsf(denom) > 1e-6f) {
				s = Clamp((b * f - c * e) / denom, 0.0f, 1.0f);
			}
			else {
				s = 0.0f;
			}

			t = (b * s + f) / e;

			if (t < 0.0f) {
				t = 0.0f;
				s = Clamp(-c / a, 0.0f, 1.0f);
			}
			else if (t > 1.0f) {
				t = 1.0f;
				s = Clamp((b - c) / a, 0.0f, 1.0f);
			}
		}
	}

	Vector c1 = Add(p1, Multiply(s, d1));
	Vector c2 = Add(p2, Multiply(t, d2));
	return LengthSq(Subtract(c1, c2));
}

Vector EvaluateBezier(const BezierCurve& curve, float t) {
	float u = 1.0f - t;
	Vector p;

	p.x = u * u * curve.p0.x + 2.0f * u * t * curve.p1.x + t * t * curve.p2.x;
	p.y = u * u * curve.p0.y + 2.0f * u * t * curve.p1.y + t * t * curve.p2.y;
	p.z = u * u * curve.p0.z + 2.0f * u * t * curve.p1.z + t * t * curve.p2.z;

	return p;
}

// 自己交差判定
bool IsSelfIntersectionBezier(const BezierCurve& curve, int subdivisions, float thresholdDistance) {
	float thresholdSq = thresholdDistance * thresholdDistance;
	const int MAX_SUBDIVISIONS = 128;
	if (subdivisions > MAX_SUBDIVISIONS) subdivisions = MAX_SUBDIVISIONS;

	Vector points[MAX_SUBDIVISIONS + 1];
	for (int i = 0; i <= subdivisions; ++i) {
		points[i] = EvaluateBezier(curve, (float)i / (float)subdivisions);
	}

	for (int i = 0; i < subdivisions; ++i) {
		Vector p1 = points[i];
		Vector q1 = points[i + 1];
		for (int j = i + 2; j < subdivisions; ++j) {
			Vector p2 = points[j];
			Vector q2 = points[j + 1];
			if (ClosestDistanceSegmentToSegmentSq(p1, q1, p2, q2) <= thresholdSq) {
				return true;
			}
		}
	}
	return false;
}

// --- 描画関数 ---

void DrawGrid(Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix, Matrix4x4 viewportMatrix) {
	const float kGridHalfWidth = 2.0f;
	const int kSubdivision = 10;
	float gridScale = kGridHalfWidth * 2.0f / (float)kSubdivision;

	for (int i = 0; i <= kSubdivision; ++i) {
		float offset = -kGridHalfWidth + (float)i * gridScale;

		Vector zStart = { offset, 0.0f, -kGridHalfWidth };
		Vector zEnd = { offset, 0.0f, kGridHalfWidth };
		Vector xStart = { -kGridHalfWidth, 0.0f, offset };
		Vector xEnd = { kGridHalfWidth, 0.0f, offset };

		Vector pZStart = Transform3DTo2D(zStart, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
		Vector pZEnd = Transform3DTo2D(zEnd, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
		Vector pXStart = Transform3DTo2D(xStart, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
		Vector pXEnd = Transform3DTo2D(xEnd, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);

		uint32_t color = (offset == 0.0f) ? 0xFFFFFFFF : 0x888888FF;

		Novice::DrawLine((int)pZStart.x, (int)pZStart.y, (int)pZEnd.x, (int)pZEnd.y, color);
		Novice::DrawLine((int)pXStart.x, (int)pXStart.y, (int)pXEnd.x, (int)pXEnd.y, color);
	}
}

void DrawBezier(const BezierCurve& curve, Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix, Matrix4x4 viewportMatrix, uint32_t color, int subdivisions) {
	Vector previousPoint = curve.p0;

	for (int i = 1; i <= subdivisions; ++i) {
		float t = (float)i / (float)subdivisions;
		Vector currentPoint = EvaluateBezier(curve, t);

		Vector pStart = Transform3DTo2D(previousPoint, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
		Vector pEnd = Transform3DTo2D(currentPoint, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);

		Novice::DrawLine((int)pStart.x, (int)pStart.y, (int)pEnd.x, (int)pEnd.y, color);

		previousPoint = currentPoint;
	}
}

// --- メイン関数 ---
const char kWindowTitle[] = "3次元衝突判定（1本の曲線 - 補助線非表示版）";

int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {

	const int kScreenWidth = 1280;
	const int kScreenHeight = 720;
	Novice::Initialize(kWindowTitle, kScreenWidth, kScreenHeight);

	// カメラ設定（斜め上から見下ろし）
	Vector cameraScale = { 1.0f, 1.0f, 1.0f };
	Vector cameraRotate = { 0.785f, 0.0f, 0.0f };
	Vector cameraTranslate = { 0.0f, 3.5f, -4.5f };

	// コンパクトなベジェ曲線
	BezierCurve curve = {
		{ -0.6f,  0.0f,  0.0f }, // p0 (controlPoints[0])
		{  0.6f,  1.0f,  0.2f }, // p1 (controlPoints[1])
		{ -0.6f,  1.0f, -0.2f }, // p2 (controlPoints[2])
	};

	int subdivisions = 48;
	float collisionThreshold = 0.05f;

	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	int prevMouseX, prevMouseY;
	Novice::GetMousePosition(&prevMouseX, &prevMouseY);

	while (Novice::ProcessMessage() == 0) {
		Novice::BeginFrame();

		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		// --- マウスによるカメラ操作処理 ---
		int mouseX, mouseY;
		Novice::GetMousePosition(&mouseX, &mouseY);
		int mouseDx = mouseX - prevMouseX;
		int mouseDy = mouseY - prevMouseY;
		int wheel = Novice::GetWheel();

		if (!ImGui::GetIO().WantCaptureMouse) {
			if (Novice::IsPressMouse(0)) {
				const float rotateSpeed = 0.005f;
				cameraRotate.x += (float)mouseDy * rotateSpeed;
				cameraRotate.y += (float)mouseDx * rotateSpeed;
			}
			if (Novice::IsPressMouse(2)) {
				const float panSpeed = 0.01f;
				float sensitivity = panSpeed * fabsf(cameraTranslate.z);
				cameraTranslate.x -= (float)mouseDx * sensitivity * cosf(cameraRotate.y);
				cameraTranslate.x -= (float)mouseDy * sensitivity * sinf(cameraRotate.y) * sinf(cameraRotate.x);
				cameraTranslate.y += (float)mouseDy * sensitivity * cosf(cameraRotate.x);
				cameraTranslate.z -= (float)mouseDx * sensitivity * sinf(cameraRotate.y);
				cameraTranslate.z += (float)mouseDy * sensitivity * cosf(cameraRotate.y) * sinf(cameraRotate.x);
			}
			if (wheel != 0) {
				const float zoomSpeed = 0.1f;
				cameraTranslate.z += (float)wheel * zoomSpeed;
			}
		}

		prevMouseX = mouseX;
		prevMouseY = mouseY;

		//====================
		// ImGui
		//====================
		ImGui::Begin("Window");
		ImGui::SetWindowSize(ImVec2(320, 140), ImGuiCond_Once);

		ImGui::DragFloat3("controlPoints[0]", &curve.p0.x, 0.01f);
		ImGui::DragFloat3("controlPoints[1]", &curve.p1.x, 0.01f);
		ImGui::DragFloat3("controlPoints[2]", &curve.p2.x, 0.01f);

		ImGui::End();

		// --- 自己交差判定の実行 ---
		bool isSelfColliding = IsSelfIntersectionBezier(curve, subdivisions, collisionThreshold);
		uint32_t curveColor = isSelfColliding ? 0xFF0000FF : 0x00FFFFFF;

		// 行列計算
		Matrix4x4 cameraWorldMatrix = MakeAffineMatrix(cameraScale, cameraRotate, cameraTranslate);
		Matrix4x4 viewMatrix = Inverse(cameraWorldMatrix);
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, (float)kScreenWidth / (float)kScreenHeight, 0.1f, 100.0f);
		Matrix4x4 viewportMatrix = MakeViewportMatrix(0.0f, 0.0f, (float)kScreenWidth, (float)kScreenHeight, 0.0f, 1.0f);

		// --- 描画処理 ---
		DrawGrid(viewMatrix, projectionMatrix, viewportMatrix);
		// 【変更】DrawControlPoints(curve, ...) をコメントアウト（黒い補助線を削除）
		DrawBezier(curve, viewMatrix, projectionMatrix, viewportMatrix, curveColor, subdivisions);

		Novice::EndFrame();

		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	Novice::Finalize();
	return 0;
}