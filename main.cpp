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

// --- 点と線分の距離演算 ---
// 点 point から、線分(originから終点diffまで)への最短距離と、その線分上の最近傍点closestPointを求める
float DistancePointToSegment(Vector point, Vector segmentStart, Vector segmentEnd, Vector& outClosestPoint) {
	Vector ab = Subtract(segmentEnd, segmentStart);
	Vector ap = Subtract(point, segmentStart);

	float abLenSq = Dot(ab, ab);
	if (abLenSq == 0.0f) {
		outClosestPoint = segmentStart;
		return Length(ap);
	}

	// 射影比率 t を計算して 0.0 ～ 1.0 にクランプ
	float t = Dot(ap, ab) / abLenSq;
	if (t < 0.0f) t = 0.0f;
	if (t > 1.0f) t = 1.0f;

	// 線分上の最も近い点
	outClosestPoint = Add(segmentStart, Multiply(t, ab));

	return Length(Subtract(point, outClosestPoint));
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

const char kWindowTitle[] = "LC1D_28_ワタナベ_アヤト_点と線分の距離";

int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {

	const int kScreenWidth = 1280;
	const int kScreenHeight = 720;
	Novice::Initialize(kWindowTitle, kScreenWidth, kScreenHeight);

	// カメラ設定
	Vector cameraScale = { 1.0f, 1.0f, 1.0f };
	Vector cameraRotate = { 0.26f, 0.0f, 0.0f };
	Vector cameraTranslate = { 0.0f, 1.5f, -5.0f };

	// 点（球体の中心として表現）
	Vector pointPos = { 0.5f, 0.8f, 0.0f };
	float sphereRadius = 0.05f; // 点の大きさを表す最小限の半径

	// 線分の設定（始点と終点）
	Vector segmentStart = { -1.0f, 0.2f, -0.5f };
	Vector segmentEnd = { 1.0f, 0.5f, 0.5f };

	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	while (Novice::ProcessMessage() == 0) {
		Novice::BeginFrame();

		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		// 点と線分の最短距離および、線分上の最短座標の計算
		Vector closestPoint{};
		float distance = DistancePointToSegment(pointPos, segmentStart, segmentEnd, closestPoint);

		// --- ImGuiによるコントロールパネル ---
		ImGui::Begin("Distance Control Panel");
		
		if (ImGui::CollapsingHeader("Camera Control", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::DragFloat3("Camera Pos", &cameraTranslate.x, 0.05f);
			ImGui::DragFloat3("Camera Rot", &cameraRotate.x, 0.01f);
		}
		
		if (ImGui::CollapsingHeader("Point (Sphere) Control", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::DragFloat3("Point Position", &pointPos.x, 0.02f);
		}
		
		if (ImGui::CollapsingHeader("Segment Control", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::DragFloat3("Segment Start (A)", &segmentStart.x, 0.02f);
			ImGui::DragFloat3("Segment End (B)", &segmentEnd.x, 0.02f);
		}

		ImGui::Separator();
		// 計算結果をImGui上にリアルタイム表示
		ImGui::Text("Calculated Distance: %.4f", distance);
		ImGui::Text("Closest Point on Wire: (%.2f, %.2f, %.2f)", closestPoint.x, closestPoint.y, closestPoint.z);
		
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

		// 2. 線分（有限の線）の描画（赤色）
		Vector pStart = Transform3DTo2D(segmentStart, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
		Vector pEnd = Transform3DTo2D(segmentEnd, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
		Novice::DrawLine((int)pStart.x, (int)pStart.y, (int)pEnd.x, (int)pEnd.y, 0xFF0000FF);

		// 3. 判定する「点」の描画（今回は小さな球体として描画・青色）
		DrawSphere(pointPos, sphereRadius, viewMatrix, projectionMatrix, viewportMatrix, 0x00FFFFFF);

		// 4. 最短距離を示す垂線の描画（点から線分上の最短座標まで・緑色）
		Vector pPoint = Transform3DTo2D(pointPos, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
		Vector pClosest = Transform3DTo2D(closestPoint, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
		Novice::DrawLine((int)pPoint.x, (int)pPoint.y, (int)pClosest.x, (int)pClosest.y, 0x00FF00FF);

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