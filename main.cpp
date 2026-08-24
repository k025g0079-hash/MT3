#include <Novice.h>
#include <math.h>
#include <assert.h>
#include <utility> 
#include <algorithm> 
#include <vector>


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

// 2次ベジェ曲線構造体
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

// 【要件対応】線形補間（Lerp）関数
Vector Lerp(const Vector& v1, const Vector& v2, float t) {
	return Add(v1, Multiply(t, Subtract(v2, v1)));
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

// 【要件対応】Lerp関数を使用したベジェ曲線の評価
Vector EvaluateBezier(const Vector& p0, const Vector& p1, const Vector& p2, float t) {
	Vector p01 = Lerp(p0, p1, t);
	Vector p12 = Lerp(p1, p2, t);
	return Lerp(p01, p12, t);
}

// 構造体オーバーロード用のラッパー
Vector EvaluateBezier(const BezierCurve& curve, float t) {
	return EvaluateBezier(curve.p0, curve.p1, curve.p2, t);
}

bool IsSelfIntersectionBezier(const BezierCurve& curve, int subdivisions, float thresholdDistance) {
	if (subdivisions < 4) return false;

	float thresholdSq = thresholdDistance * thresholdDistance;

	std::vector<Vector> points(subdivisions + 1);
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
void DrawGrid(const Matrix4x4& vpVpMatrix) {
	const float kGridHalfWidth = 2.0f;
	const int kSubdivision = 10;
	float gridScale = kGridHalfWidth * 2.0f / (float)kSubdivision;

	for (int i = 0; i <= kSubdivision; ++i) {
		float offset = -kGridHalfWidth + (float)i * gridScale;

		Vector zStart = { offset, 0.0f, -kGridHalfWidth };
		Vector zEnd = { offset, 0.0f, kGridHalfWidth };
		Vector xStart = { -kGridHalfWidth, 0.0f, offset };
		Vector xEnd = { kGridHalfWidth, 0.0f, offset };

		Vector pZStart = Transform(zStart, vpVpMatrix);
		Vector pZEnd = Transform(zEnd, vpVpMatrix);
		Vector pXStart = Transform(xStart, vpVpMatrix);
		Vector pXEnd = Transform(xEnd, vpVpMatrix);

		uint32_t color = (offset == 0.0f) ? 0xFFFFFFFF : 0x888888FF;

		Novice::DrawLine((int)pZStart.x, (int)pZStart.y, (int)pZEnd.x, (int)pZEnd.y, color);
		Novice::DrawLine((int)pXStart.x, (int)pXStart.y, (int)pXEnd.x, (int)pXEnd.y, color);
	}
}

// 【要件対応】指定された個別の制御点を受け取るシグネチャに変更
void DrawBezier(const Vector& controlPoint0, const Vector& controlPoint1, const Vector& controlPoint2, const Matrix4x4& vpVpMatrix, uint32_t color, int subdivisions) {
	Vector previousPoint = controlPoint0;

	for (int i = 1; i <= subdivisions; ++i) {
		float t = (float)i / (float)subdivisions;
		Vector currentPoint = EvaluateBezier(controlPoint0, controlPoint1, controlPoint2, t);

		Vector pStart = Transform(previousPoint, vpVpMatrix);
		Vector pEnd = Transform(currentPoint, vpVpMatrix);

		Novice::DrawLine((int)pStart.x, (int)pStart.y, (int)pEnd.x, (int)pEnd.y, color);

		previousPoint = currentPoint;
	}
}

// 【要件対応】コントロールポイント可視化用の球描画関数
void DrawSphere(const Vector& center, float radius, const Matrix4x4& vpVpMatrix, uint32_t color) {
	const int kSubdivision = 12;
	const float PI = 3.14159265358979323846f;

	const float kLatEvery = PI / kSubdivision;
	const float kLonEvery = PI * 2.0f / kSubdivision;

	for (int latIndex = 0; latIndex < kSubdivision; ++latIndex) {
		float lat = -(float)PI / 2.0f + kLatEvery * latIndex;
		for (int lonIndex = 0; lonIndex < kSubdivision; ++lonIndex) {
			float lon = kLonEvery * lonIndex;

			Vector a = {
				center.x + radius * cosf(lat) * cosf(lon),
				center.y + radius * sinf(lat),
				center.z + radius * cosf(lat) * sinf(lon)
			};
			Vector b = {
				center.x + radius * cosf(lat + kLatEvery) * cosf(lon),
				center.y + radius * sinf(lat + kLatEvery),
				center.z + radius * cosf(lat + kLatEvery) * sinf(lon)
			};
			Vector c = {
				center.x + radius * cosf(lat) * cosf(lon + kLonEvery),
				center.y + radius * sinf(lat),
				center.z + radius * cosf(lat) * sinf(lon + kLonEvery)
			};

			Vector pa = Transform(a, vpVpMatrix);
			Vector pb = Transform(b, vpVpMatrix);
			Vector pc = Transform(c, vpVpMatrix);

			Novice::DrawLine((int)pa.x, (int)pa.y, (int)pb.x, (int)pb.y, color);
			Novice::DrawLine((int)pa.x, (int)pa.y, (int)pc.x, (int)pc.y, color);
		}
	}
}

// --- メイン関数 ---
const char kWindowTitle[] = "3次元衝突判定（1本の曲線 - コントロールポイント可視化版）";

int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {

	const int kScreenWidth = 1280;
	const int kScreenHeight = 720;
	Novice::Initialize(kWindowTitle, kScreenWidth, kScreenHeight);

	// カメラ設定
	Vector cameraScale = { 1.0f, 1.0f, 1.0f };
	Vector cameraRotate = { 0.785f, 0.0f, 0.0f };
	Vector cameraTranslate = { 0.0f, 3.5f, -4.5f };

	// ベジェ曲線
	BezierCurve curve = {
		{ -0.6f,  0.0f,  0.0f }, // p0
		{  0.6f,  1.0f,  0.2f }, // p1
		{ -0.6f,  1.0f, -0.2f }, // p2
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

		// --- マウス操作 ---
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

		// --- ImGui ---
		ImGui::Begin("Window");
		ImGui::SetWindowSize(ImVec2(320, 160), ImGuiCond_Once);

		ImGui::DragFloat3("controlPoints[0]", &curve.p0.x, 0.01f);
		ImGui::DragFloat3("controlPoints[1]", &curve.p1.x, 0.01f);
		ImGui::DragFloat3("controlPoints[2]", &curve.p2.x, 0.01f);
		ImGui::SliderInt("Subdivisions", &subdivisions, 10, 128);

		ImGui::End();

		// --- 判定 ---
		bool isSelfColliding = IsSelfIntersectionBezier(curve, subdivisions, collisionThreshold);
		uint32_t curveColor = isSelfColliding ? 0xFF0000FF : 0x00FFFFFF;

		// --- 行列計算 ---
		Matrix4x4 cameraWorldMatrix = MakeAffineMatrix(cameraScale, cameraRotate, cameraTranslate);
		Matrix4x4 viewMatrix = Inverse(cameraWorldMatrix);
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, (float)kScreenWidth / (float)kScreenHeight, 0.1f, 100.0f);
		Matrix4x4 viewportMatrix = MakeViewportMatrix(0.0f, 0.0f, (float)kScreenWidth, (float)kScreenHeight, 0.0f, 1.0f);
		Matrix4x4 vpVpMatrix = Multiply(viewMatrix, Multiply(projectionMatrix, viewportMatrix));

		// --- 描画処理 ---
		DrawGrid(vpVpMatrix);

		// 【要件対応】修正したDrawBezier関数の呼び出し
		DrawBezier(curve.p0, curve.p1, curve.p2, vpVpMatrix, curveColor, subdivisions);

		// 【要件対応】コントロールポイントの球描画
		const float pointRadius = 0.05f;
		DrawSphere(curve.p0, pointRadius, vpVpMatrix, 0x00FF00FF); // Green
		DrawSphere(curve.p1, pointRadius, vpVpMatrix, 0xFFFF00FF); // Yellow
		DrawSphere(curve.p2, pointRadius, vpVpMatrix, 0x00FF00FF); // Green

		Novice::EndFrame();

		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	Novice::Finalize();
	return 0;
}