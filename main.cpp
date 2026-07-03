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

// 線分（有限の線）構造体
struct Segment {
	Vector origin; // 始点
	Vector diff;   // 始点から終点へのベクトル（終点 - 始点）
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
		if (fabsf(pivot) < 1e-6f) pivot = 1e-6f; // 簡易的なゼロ除算対策
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

// --- ベクトルと行列による座標変換 ---
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

// AABBと線分（Segment）の衝突判定：スラブ判定法
bool IsCollisionAABBToSegment(AABB aabb, Segment segment) {
	float t_min = 0.0f;
	float t_max = 1.0f; // 線分なので媒介変数 t の有効範囲は 0.0 ～ 1.0

	// 各軸の要素にアクセスしやすくするため、配列として展開
	float origin[] = { segment.origin.x, segment.origin.y, segment.origin.z };
	float diff[] = { segment.diff.x, segment.diff.y, segment.diff.z };
	float aabb_min[] = { aabb.min.x, aabb.min.y, aabb.min.z };
	float aabb_max[] = { aabb.max.x, aabb.max.y, aabb.max.z };

	for (int i = 0; i < 3; ++i) {
		// 軸に平行（方向ベクトルの成分がほぼゼロ）な場合の例外処理
		if (fabsf(diff[i]) < 1e-6f) {
			// 線分の始点がAABBのスラブ外にあるなら絶対に当たらない
			if (origin[i] < aabb_min[i] || origin[i] > aabb_max[i]) {
				return false;
			}
		}
		else {
			// 手前と奥のスラブへの衝突時間 t を計算
			float t1 = (aabb_min[i] - origin[i]) / diff[i];
			float t2 = (aabb_max[i] - origin[i]) / diff[i];

			// t1 が手前、t2 が奥になるように順序を揃える
			if (t1 > t2) {
				std::swap(t1, t2);
			}

			// 全軸共通の通過可能区間（重なり）を更新
			t_min = (std::max)(t_min, t1);
			t_max = (std::min)(t_max, t2);

			// 区間が逆転（矛盾）したら、その時点で衝突していない
			if (t_min > t_max) {
				return false;
			}
		}
	}

	return true;
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

// 線分の描画関数
void DrawSegment(Segment segment, Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix, Matrix4x4 viewportMatrix, uint32_t color) {
	Vector start = segment.origin;
	Vector end = Add(segment.origin, segment.diff); // 始点 + 差分ベクトル = 終点

	Vector pStart = Transform3DTo2D(start, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Vector pEnd = Transform3DTo2D(end, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);

	Novice::DrawLine((int)pStart.x, (int)pStart.y, (int)pEnd.x, (int)pEnd.y, color);
}

// --- メイン関数 ---
const char kWindowTitle[] = "3次元衝突判定（AABBと線分）- マウス操作対応";

int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {

	const int kScreenWidth = 1280;
	const int kScreenHeight = 720;
	Novice::Initialize(kWindowTitle, kScreenWidth, kScreenHeight);

	// カメラ設定（初期状態：中心正面アングル）
	Vector cameraScale = { 1.0f, 1.0f, 1.0f };
	Vector cameraRotate = { 0.0f, 0.0f, 0.0f };
	Vector cameraTranslate = { 0.0f, 0.5f, -4.5f };

	// AABB の初期設定
	AABB aabb = {
		{ -0.5f, 0.0f, -0.5f }, // min
		{  0.5f, 1.0f,  0.5f }  // max
	};

	// 線分（Segment）の初期設定
	Segment segment = {
		{ 0.0f, 1.5f, -1.0f }, // origin (始点)
		{ 0.0f, -2.0f, 2.0f }  // diff (方向・長さベクトル)
	};

	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	// マウス操作用の変数
	int prevMouseX, prevMouseY;
	Novice::GetMousePosition(&prevMouseX, &prevMouseY);

	while (Novice::ProcessMessage() == 0) {
		Novice::BeginFrame();

		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		// --- マウスによるカメラ操作処理 ---
		int mouseX, mouseY;
		Novice::GetMousePosition(&mouseX, &mouseY);
		int mouseDx = mouseX - prevMouseX; // マウスの移動量(X)
		int mouseDy = mouseY - prevMouseY; // マウスの移動量(Y)
		int wheel = Novice::GetWheel();   // ホイールの回転量

		// ImGuiのウィンドウ上にマウスがない場合のみ、画面操作を受け付ける
		if (!ImGui::GetIO().WantCaptureMouse) {

			// 1. 左ドラッグ：回転 (Pitch & Yaw)
			if (Novice::IsPressMouse(0)) {
				const float rotateSpeed = 0.005f; // 回転感度
				cameraRotate.x += (float)mouseDy * rotateSpeed; // 上下ドラッグでX軸回転
				cameraRotate.y += (float)mouseDx * rotateSpeed; // 左右ドラッグでY軸回転
			}

			// 2. 中ドラッグ（ホイールクリック）：平行移動 (Pan)
			if (Novice::IsPressMouse(2)) {
				const float panSpeed = 0.01f; // 平行移動感度
				// カメラの向きに合わせて移動方向を補正する簡易実装
				float sensitivity = panSpeed * fabsf(cameraTranslate.z); // 距離に応じて移動量を調整
				cameraTranslate.x -= (float)mouseDx * sensitivity * cosf(cameraRotate.y);
				cameraTranslate.x -= (float)mouseDy * sensitivity * sinf(cameraRotate.y) * sinf(cameraRotate.x);
				cameraTranslate.y += (float)mouseDy * sensitivity * cosf(cameraRotate.x);
				cameraTranslate.z -= (float)mouseDx * sensitivity * sinf(cameraRotate.y);
				cameraTranslate.z += (float)mouseDy * sensitivity * cosf(cameraRotate.y) * sinf(cameraRotate.x);
			}

			// 3. ホイール回転：前後移動 (Zoom)
			if (wheel != 0) {
				const float zoomSpeed = 0.1f; // ズーム感度
				cameraTranslate.z += (float)wheel * zoomSpeed;
			}
		}

		// 次フレームのために現在のマウス位置を保存
		prevMouseX = mouseX;
		prevMouseY = mouseY;


		ImGui::Begin("Window");

		ImGui::DragFloat3("aabb.min", &aabb.min.x, 0.01f);
		ImGui::DragFloat3("aabb.max", &aabb.max.x, 0.01f);

		ImGui::DragFloat3("segment.origin", &segment.origin.x, 0.01f);
		ImGui::DragFloat3("segment.diff", &segment.diff.x, 0.01f);

		ImGui::End();

		// 【対策】Windows.hのマクロ競合を防ぐため、(std::min) と (std::max) のようにカッコで囲んでいます
		aabb = {
			{ (std::min)(aabb.min.x, aabb.max.x), (std::min)(aabb.min.y, aabb.max.y), (std::min)(aabb.min.z, aabb.max.z) },
			{ (std::max)(aabb.min.x, aabb.max.x), (std::max)(aabb.min.y, aabb.max.y), (std::max)(aabb.min.z, aabb.max.z) }
		};

		// 衝突判定を実行
		bool isColliding = IsCollisionAABBToSegment(aabb, segment);

		// 状態に応じたカラーの選定
		uint32_t colorAABB = isColliding ? 0xFF0000FF : 0xFFFFFFFF;     // 衝突:赤 / 非衝突:白
		uint32_t colorSegment = isColliding ? 0xFF0000FF : 0x00FFFFFF;  // 衝突:赤 / 非衝突:シアン（水色）

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

		// 3. 線分
		DrawSegment(segment, viewMatrix, projectionMatrix, viewportMatrix, colorSegment);

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