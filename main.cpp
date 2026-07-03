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

struct AABB {
	Vector min; // 最小座標
	Vector max; // 最大座標
};

// 球構造体の追加
struct Sphere {
	Vector center; // 中心点
	float radius;  // 半径
};

// --- ベクトル演算 ---
Vector Add(Vector v1, Vector v2) { return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z }; }
Vector Subtract(Vector v1, Vector v2) { return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z }; }
Vector Multiply(float k, Vector v) { return { k * v.x, k * v.y, k * v.z }; }
float Length(Vector v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }
Vector Normalize(Vector v) {
	float len = Length(v);
	if (len == 0.0f) return { 0, 0, 0 };
	return { v.x / len, v.y / len, v.z / len };
}

// クランプ関数の定義（AABBの最近傍点を求める用）
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
		assert(fabsf(pivot) > 1e-6f);
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

// --- 衝突判定関数 ---

// AABBと球の衝突判定
bool IsCollisionAABBToSphere(AABB aabb, Sphere sphere) {
	// AABB上の点の中で、球の中心に最も近い点（最近傍点）を求める
	Vector closestPoint;
	closestPoint.x = Clamp(sphere.center.x, aabb.min.x, aabb.max.x);
	closestPoint.y = Clamp(sphere.center.y, aabb.min.y, aabb.max.y);
	closestPoint.z = Clamp(sphere.center.z, aabb.min.z, aabb.max.z);

	// 最近傍点と球の中心との距離を計算
	Vector diff = Subtract(closestPoint, sphere.center);
	float distance = Length(diff);

	// 距離が球の半径以下であれば衝突している
	return distance <= sphere.radius;
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

void DrawAABB(AABB aabb, Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix, Matrix4x4 viewportMatrix, uint32_t color) {
	Vector vertices[8] = {
		{ aabb.min.x, aabb.min.y, aabb.min.z },
		{ aabb.max.x, aabb.min.y, aabb.min.z },
		{ aabb.min.x, aabb.max.y, aabb.min.z },
		{ aabb.max.x, aabb.max.y, aabb.min.z },
		{ aabb.min.x, aabb.min.y, aabb.max.z },
		{ aabb.max.x, aabb.min.y, aabb.max.z },
		{ aabb.min.x, aabb.max.y, aabb.max.z },
		{ aabb.max.x, aabb.max.y, aabb.max.z }
	};

	Vector screenVertices[8];
	for (int i = 0; i < 8; ++i) {
		screenVertices[i] = Transform3DTo2D(vertices[i], MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	}

	// 手前の面
	Novice::DrawLine((int)screenVertices[0].x, (int)screenVertices[0].y, (int)screenVertices[1].x, (int)screenVertices[1].y, color);
	Novice::DrawLine((int)screenVertices[1].x, (int)screenVertices[1].y, (int)screenVertices[3].x, (int)screenVertices[3].y, color);
	Novice::DrawLine((int)screenVertices[3].x, (int)screenVertices[3].y, (int)screenVertices[2].x, (int)screenVertices[2].y, color);
	Novice::DrawLine((int)screenVertices[2].x, (int)screenVertices[2].y, (int)screenVertices[0].x, (int)screenVertices[0].y, color);

	// 奥の面
	Novice::DrawLine((int)screenVertices[4].x, (int)screenVertices[4].y, (int)screenVertices[5].x, (int)screenVertices[5].y, color);
	Novice::DrawLine((int)screenVertices[5].x, (int)screenVertices[5].y, (int)screenVertices[7].x, (int)screenVertices[7].y, color);
	Novice::DrawLine((int)screenVertices[7].x, (int)screenVertices[7].y, (int)screenVertices[6].x, (int)screenVertices[6].y, color);
	Novice::DrawLine((int)screenVertices[6].x, (int)screenVertices[6].y, (int)screenVertices[4].x, (int)screenVertices[4].y, color);

	// 縦の柱
	Novice::DrawLine((int)screenVertices[0].x, (int)screenVertices[0].y, (int)screenVertices[4].x, (int)screenVertices[4].y, color);
	Novice::DrawLine((int)screenVertices[1].x, (int)screenVertices[1].y, (int)screenVertices[5].x, (int)screenVertices[5].y, color);
	Novice::DrawLine((int)screenVertices[2].x, (int)screenVertices[2].y, (int)screenVertices[6].x, (int)screenVertices[6].y, color);
	Novice::DrawLine((int)screenVertices[3].x, (int)screenVertices[3].y, (int)screenVertices[7].x, (int)screenVertices[7].y, color);
}

// 球をワイヤーフレーム（3軸の円）で描画する関数
void DrawSphere(Sphere sphere, Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix, Matrix4x4 viewportMatrix, uint32_t color) {
	const int kSubdivision = 16; // 円の分割数
	const float pi = 3.14159265f;

	for (int i = 0; i < kSubdivision; ++i) {
		float angle1 = (float)i * 2.0f * pi / (float)kSubdivision;
		float angle2 = (float)(i + 1) * 2.0f * pi / (float)kSubdivision;

		// 1. XY平面上の円
		Vector pXY1 = { sphere.center.x + sphere.radius * cosf(angle1), sphere.center.y + sphere.radius * sinf(angle1), sphere.center.z };
		Vector pXY2 = { sphere.center.x + sphere.radius * cosf(angle2), sphere.center.y + sphere.radius * sinf(angle2), sphere.center.z };
		Vector sXY1 = Transform3DTo2D(pXY1, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
		Vector sXY2 = Transform3DTo2D(pXY2, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
		Novice::DrawLine((int)sXY1.x, (int)sXY1.y, (int)sXY2.x, (int)sXY2.y, color);

		// 2. XZ平面上の円
		Vector pXZ1 = { sphere.center.x + sphere.radius * cosf(angle1), sphere.center.y, sphere.center.z + sphere.radius * sinf(angle1) };
		Vector pXZ2 = { sphere.center.x + sphere.radius * cosf(angle2), sphere.center.y, sphere.center.z + sphere.radius * sinf(angle2) };
		Vector sXZ1 = Transform3DTo2D(pXZ1, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
		Vector sXZ2 = Transform3DTo2D(pXZ2, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
		Novice::DrawLine((int)sXZ1.x, (int)sXZ1.y, (int)sXZ2.x, (int)sXZ2.y, color);

		// 3. YZ平面上の円
		Vector pYZ1 = { sphere.center.x, sphere.center.y + sphere.radius * cosf(angle1), sphere.center.z + sphere.radius * sinf(angle1) };
		Vector pYZ2 = { sphere.center.x, sphere.center.y + sphere.radius * cosf(angle2), sphere.center.z + sphere.radius * sinf(angle2) };
		Vector sYZ1 = Transform3DTo2D(pYZ1, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
		Vector sYZ2 = Transform3DTo2D(pYZ2, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
		Novice::DrawLine((int)sYZ1.x, (int)sYZ1.y, (int)sYZ2.x, (int)sYZ2.y, color);
	}
}

// --- メイン関数 ---
const char kWindowTitle[] = "3次元衝突判定（AABBと球）";

int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {

	const int kScreenWidth = 1280;
	const int kScreenHeight = 720;
	Novice::Initialize(kWindowTitle, kScreenWidth, kScreenHeight);

	// カメラ設定
	Vector cameraScale = { 1.0f, 1.0f, 1.0f };
	Vector cameraRotate = { 0.35f, -0.6f, 0.0f };
	Vector cameraTranslate = { 1.5f, 2.5f, -4.5f };

	// AABB の初期設定
	AABB aabb = {
		{ -0.5f, 0.0f, -0.5f }, // min
		{  0.5f, 1.0f,  0.5f }  // max
	};

	// 球（Sphere）の初期設定
	Sphere sphere = {
		{ 0.7f, 0.5f, 0.7f }, // center
		0.4f                  // radius
	};

	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	while (Novice::ProcessMessage() == 0) {
		Novice::BeginFrame();

		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		// --- ImGuiによるコントロールパネル ---
		ImGui::Begin("Collision Control Panel");

		if (ImGui::CollapsingHeader("Camera Control", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::DragFloat3("Camera Pos", &cameraTranslate.x, 0.05f);
			ImGui::DragFloat3("Camera Rot", &cameraRotate.x, 0.01f);
		}

		if (ImGui::CollapsingHeader("AABB Control", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::DragFloat3("AABB Min", &aabb.min.x, 0.02f);
			ImGui::DragFloat3("AABB Max", &aabb.max.x, 0.02f);
		}

		if (ImGui::CollapsingHeader("Sphere Control", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::DragFloat3("Sphere Center", &sphere.center.x, 0.02f);
			ImGui::DragFloat("Sphere Radius", &sphere.radius, 0.01f, 0.01f, 5.0f);
		}

		ImGui::Separator();

		// 【対策】Windows.hのマクロ競合を防ぐため、(std::min) と (std::max) のようにカッコで囲んでいます
		aabb = {
			{ (std::min)(aabb.min.x, aabb.max.x), (std::min)(aabb.min.y, aabb.max.y), (std::min)(aabb.min.z, aabb.max.z) },
			{ (std::max)(aabb.min.x, aabb.max.x), (std::max)(aabb.min.y, aabb.max.y), (std::max)(aabb.min.z, aabb.max.z) }
		};

		// 衝突判定
		bool isColliding = IsCollisionAABBToSphere(aabb, sphere);

		// 状態に応じたカラー
		uint32_t colorAABB = isColliding ? 0xFF0000FF : 0xFFFFFFFF;   // 衝突:赤 / 非衝突:白
		uint32_t colorSphere = isColliding ? 0xFF0000FF : 0x00FF00FF; // 衝突:赤 / 非衝突:緑

		// 結果表示
		ImGui::Text("--- Result ---");
		if (isColliding) {
			ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "STATUS: COLLIDING!");
		}
		else {
			ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "STATUS: NO COLLISION");
		}

		ImGui::End();

		// 行列の生成
		Matrix4x4 cameraWorldMatrix = MakeAffineMatrix(cameraScale, cameraRotate, cameraTranslate);
		Matrix4x4 viewMatrix = Inverse(cameraWorldMatrix);
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, (float)kScreenWidth / (float)kScreenHeight, 0.1f, 100.0f);
		Matrix4x4 viewportMatrix = MakeViewportMatrix(0.0f, 0.0f, (float)kScreenWidth, (float)kScreenHeight, 0.0f, 1.0f);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		// 1. グリッド
		DrawGrid(viewMatrix, projectionMatrix, viewportMatrix);

		// 2. AABB
		DrawAABB(aabb, viewMatrix, projectionMatrix, viewportMatrix, colorAABB);

		// 3. 球
		DrawSphere(sphere, viewMatrix, projectionMatrix, viewportMatrix, colorSphere);

		///
		/// ↑描画処理ここまで
		///

		Novice::EndFrame();

		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	Novice::Finalize();
	return 0;
}