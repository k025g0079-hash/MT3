#include <Novice.h>
#include <math.h>
#include <assert.h>
#include <utility> 
#define ImGui_ImplDX12_RGB_Color
#include <imgui.h>

struct Vector {
	float x;
	float y;
	float z;
};

struct Matrix4x4
{
	float m[4][4];
};

// --- 平面の構造体 ---
struct Plane {
	Vector normal;   // 平面の法線（必ず正規化されたベクトル）
	float distance;  // 原点から平面までの最短距離
};

// --- ベクトル演算 ---
Vector Add(Vector v1, Vector v2) { return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z }; }
Vector Subtract(Vector v1, Vector v2) { return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z }; }
Vector Multiply(float k, Vector v) { return { k * v.x, k * v.y, k * v.z }; }
float Dot(Vector v1, Vector v2) { return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z; }
float Length(Vector v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }
Vector Normalize(Vector v) {
	float len = Length(v);
	if (len == 0.0f) return { 0, 0, 0 };
	return { v.x / len, v.y / len, v.z / len };
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

// --- 3D変換用行列生成 ---
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

// --- グリッド描画 ---
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

// --- 球のワイヤーフレーム描画 ---
void DrawSphere(Vector center, float radius, Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix, Matrix4x4 viewportMatrix, uint32_t color) {
	const int kSubdivision = 12;
	const float pi = 3.1415926535f;
	float latStep = pi / (float)kSubdivision;
	float lonStep = (pi * 2.0f) / (float)kSubdivision;

	Matrix4x4 worldMatrix = MakeAffineMatrix({ 1,1,1 }, { 0,0,0 }, center);

	for (int lat = 0; lat < kSubdivision; ++lat) {
		float latAngle = -pi / 2.0f + (float)lat * latStep;
		for (int lon = 0; lon < kSubdivision; ++lon) {
			float lonAngle = (float)lon * lonStep;

			Vector a = { radius * cosf(latAngle) * cosf(lonAngle), radius * sinf(latAngle), radius * cosf(latAngle) * sinf(lonAngle) };
			Vector b = { radius * cosf(latAngle) * cosf(lonAngle + lonStep), radius * sinf(latAngle), radius * cosf(latAngle) * sinf(lonAngle + lonStep) };
			Vector c = { radius * cosf(latAngle + latStep) * cosf(lonAngle), radius * sinf(latAngle + latStep), radius * cosf(latAngle + latStep) * sinf(lonAngle) };

			Vector pa = Transform3DTo2D(a, worldMatrix, viewMatrix, projectionMatrix, viewportMatrix);
			Vector pb = Transform3DTo2D(b, worldMatrix, viewMatrix, projectionMatrix, viewportMatrix);
			Vector pc = Transform3DTo2D(c, worldMatrix, viewMatrix, projectionMatrix, viewportMatrix);

			Novice::DrawLine((int)pa.x, (int)pa.y, (int)pb.x, (int)pb.y, color);
			Novice::DrawLine((int)pa.x, (int)pa.y, (int)pc.x, (int)pc.y, color);
		}
	}
}

// --- 平面のワイヤーフレーム描画関数 ---
void DrawPlane(Plane plane, Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix, Matrix4x4 viewportMatrix, uint32_t color) {
	Vector ext1, ext2;
	if (fabsf(plane.normal.x) > 0.9f) {
		ext1 = { 0.0f, 1.0f, 0.0f };
	}
	else {
		ext1 = { 1.0f, 0.0f, 0.0f };
	}
	ext1 = Normalize(Subtract(ext1, Multiply(Dot(ext1, plane.normal), plane.normal)));
	ext2 = {
		plane.normal.y * ext1.z - plane.normal.z * ext1.y,
		plane.normal.z * ext1.x - plane.normal.x * ext1.z,
		plane.normal.x * ext1.y - plane.normal.y * ext1.x
	};

	Vector center = Multiply(plane.distance, plane.normal);
	float size = 2.0f;

	Vector v0 = Add(center, Add(Multiply(-size, ext1), Multiply(-size, ext2)));
	Vector v1 = Add(center, Add(Multiply(size, ext1), Multiply(-size, ext2)));
	Vector v2 = Add(center, Add(Multiply(size, ext1), Multiply(size, ext2)));
	Vector v3 = Add(center, Add(Multiply(-size, ext1), Multiply(size, ext2)));

	Vector p0 = Transform3DTo2D(v0, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Vector p1 = Transform3DTo2D(v1, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Vector p2 = Transform3DTo2D(v2, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Vector p3 = Transform3DTo2D(v3, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);

	Novice::DrawLine((int)p0.x, (int)p0.y, (int)p1.x, (int)p1.y, color);
	Novice::DrawLine((int)p1.x, (int)p1.y, (int)p2.x, (int)p2.y, color);
	Novice::DrawLine((int)p2.x, (int)p2.y, (int)p3.x, (int)p3.y, color);
	Novice::DrawLine((int)p3.x, (int)p3.y, (int)p0.x, (int)p0.y, color);

	// 平面の中央から法線ベクトルを視覚化（マゼンタ色）
	Vector normalEnd = Add(center, Multiply(0.4f, plane.normal));
	Vector pCenter = Transform3DTo2D(center, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Vector pNormalEnd = Transform3DTo2D(normalEnd, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Novice::DrawLine((int)pCenter.x, (int)pCenter.y, (int)pNormalEnd.x, (int)pNormalEnd.y, 0xFF00FFFF);
}

// --- 平面と球の衝突判定関数 ---
bool IsCollisionPlaneToSphere(Plane plane, Vector sphereCenter, float sphereRadius, float& outDistance) {
	// 点と平面の距離の公式: d = |(Normal ・ Center) - Distance|
	float signedDistance = Dot(plane.normal, sphereCenter) - plane.distance;
	outDistance = fabsf(signedDistance);
	return outDistance <= sphereRadius;
}

const char kWindowTitle[] = "LC1D_28_ワタナベ_アヤト_3次元衝突判定（平面と球）";

int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {

	const int kScreenWidth = 1280;
	const int kScreenHeight = 720;
	Novice::Initialize(kWindowTitle, kScreenWidth, kScreenHeight);

	// カメラ設定
	Vector cameraScale = { 1.0f, 1.0f, 1.0f };
	Vector cameraRotate = { 0.26f, 0.0f, 0.0f };
	Vector cameraTranslate = { 0.0f, 1.5f, -5.0f };

	// 球の設定
	Vector sphereCenter = { 0.0f, 0.5f, 0.0f };
	float sphereRadius = 0.3f;

	// 平目の設定 (初期値は真上を向いた原点を通る床)
	Plane plane = {
		Normalize({ 0.0f, 1.0f, 0.0f }),
		0.0f
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

		// 平面と球の衝突判定
		float planeToSphereDistance = 0.0f;
		bool isColliding = IsCollisionPlaneToSphere(plane, sphereCenter, sphereRadius, planeToSphereDistance);

		// 衝突状態によって描画色を切り替える
		uint32_t colorSphere = isColliding ? 0xFF0000FF : 0x00FFFFFF; // 衝突:赤 / 非衝突:シアン
		uint32_t colorPlane = isColliding ? 0xFF0000FF : 0xFFFFFFFF; // 衝突:赤 / 非衝突:白

		// --- ImGuiによるコントロールパネル ---
		ImGui::Begin("Collision Control Panel");

		if (ImGui::CollapsingHeader("Camera Control", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::DragFloat3("Camera Pos", &cameraTranslate.x, 0.05f);
			ImGui::DragFloat3("Camera Rot", &cameraRotate.x, 0.01f);
		}

		if (ImGui::CollapsingHeader("Sphere Control", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::DragFloat3("Sphere Center", &sphereCenter.x, 0.02f);
			ImGui::DragFloat("Sphere Radius", &sphereRadius, 0.01f, 0.01f, 2.0f);
		}

		if (ImGui::CollapsingHeader("Plane Control", ImGuiTreeNodeFlags_DefaultOpen)) {
			if (ImGui::DragFloat3("Plane Normal", &plane.normal.x, 0.01f, -1.0f, 1.0f)) {
				plane.normal = Normalize(plane.normal); // 法線が変わったら再正規化
			}
			ImGui::DragFloat("Plane Distance", &plane.distance, 0.02f, -5.0f, 5.0f);
		}

		ImGui::Separator();

		// 計算結果・衝突ステータス表示
		ImGui::Text("--- Plane to Sphere ---");
		ImGui::Text("Plane-Sphere Dist: %.4f", planeToSphereDistance);
		ImGui::Text("Sphere Radius: %.4f", sphereRadius);
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

		// 1. グリッドの描画
		DrawGrid(viewMatrix, projectionMatrix, viewportMatrix);

		// 2. 平面の描画
		DrawPlane(plane, viewMatrix, projectionMatrix, viewportMatrix, colorPlane);

		// 3. 球の描画
		DrawSphere(sphereCenter, sphereRadius, viewMatrix, projectionMatrix, viewportMatrix, colorSphere);

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