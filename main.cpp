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

// 【新規】階層構造の各関節（ノード）を表す構造体
struct Joint {
	Vector position; // 3D座標
	uint32_t color;  // 部位固有の色
};

// 【変更】人間の腕をベースにしたベジェ曲線階層構造
struct ArmBezier {
	Joint shoulder; // 親ノード (制御点0): 肩 [赤]
	Joint elbow;    // 子ノード (制御点1): 肘 [緑]
	Joint hand;     // 孫ノード (制御点2): 手 [青]
};

// --- ベクトル・行列演算 ---
Vector Add(Vector v1, Vector v2) { return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z }; }
Vector Subtract(Vector v1, Vector v2) { return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z }; }
Vector Multiply(float k, Vector v) { return { k * v.x, k * v.y, k * v.z }; }
float Dot(Vector v1, Vector v2) { return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z; }
float LengthSq(Vector v) { return v.x * v.x + v.y * v.y + v.z * v.z; }
float Clamp(float value, float min, float max) { return (value < min) ? min : ((value > max) ? max : value); }

Matrix4x4 Multiply(Matrix4x4 A, Matrix4x4 B) {
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			for (int k = 0; k < 4; k++) result.m[i][j] += A.m[i][k] * B.m[k][j];
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
			if (fabsf(a[k][i]) > maxVal) { maxVal = fabsf(a[k][i]); pivotRow = k; }
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

Matrix4x4 MakeAffineMatrix(Vector scale, Vector rotate, Vector translate) {
	Matrix4x4 rotateX = { {{1.0f,0.0f,0.0f,0.0f},{0.0f,cosf(rotate.x),sinf(rotate.x),0.0f},{0.0f,-sinf(rotate.x),cosf(rotate.x),0.0f},{0.0f,0.0f,0.0f,1.0f}} };
	Matrix4x4 rotateY = { {{cosf(rotate.y),0.0f,-sinf(rotate.y),0.0f},{0.0f,1.0f,0.0f,0.0f},{sinf(rotate.y),0.0f,cosf(rotate.y),0.0f},{0.0f,0.0f,0.0f,1.0f}} };
	Matrix4x4 rotateZ = { {{cosf(rotate.z),sinf(rotate.z),0.0f,0.0f},{-sinf(rotate.z),cosf(rotate.z),0.0f,0.0f},{0.0f,0.0f,1.0f,0.0f},{0.0f,0.0f,0.0f,1.0f}} };
	Matrix4x4 rotMat = Multiply(rotateX, Multiply(rotateY, rotateZ));
	return { {
		{scale.x * rotMat.m[0][0], scale.x * rotMat.m[0][1], scale.x * rotMat.m[0][2], 0.0f},
		{scale.y * rotMat.m[1][0], scale.y * rotMat.m[1][1], scale.y * rotMat.m[1][2], 0.0f},
		{scale.z * rotMat.m[2][0], scale.z * rotMat.m[2][1], scale.z * rotMat.m[2][2], 0.0f},
		{translate.x, translate.y, translate.z, 1.0f}
	} };
}

Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspect, float nearClip, float farClip) {
	Matrix4x4 result{};
	float cot = 1.0f / tanf(fovY / 2.0f);
	result.m[0][0] = cot / aspect; result.m[1][1] = cot;
	result.m[2][2] = farClip / (farClip - nearClip); result.m[2][3] = 1.0f;
	result.m[3][2] = (-nearClip * farClip) / (farClip - nearClip);
	return result;
}

Matrix4x4 MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth) {
	return { {
		{width / 2.0f, 0.0f, 0.0f, 0.0f},
		{0.0f, -height / 2.0f, 0.0f, 0.0f},
		{0.0f, 0.0f, maxDepth - minDepth, 0.0f},
		{left + width / 2.0f, top + height / 2.0f, minDepth, 1.0f}
	} };
}

Vector Transform(Vector vector, Matrix4x4 matrix) {
	float w = vector.x * matrix.m[0][3] + vector.y * matrix.m[1][3] + vector.z * matrix.m[2][3] + matrix.m[3][3];
	if (w == 0.0f) w = 1.0f;
	return {
		(vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] + matrix.m[3][0]) / w,
		(vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] + matrix.m[3][1]) / w,
		(vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] + matrix.m[3][2]) / w
	};
}

// --- 衝突・ベジェ演算 ---
float ClosestDistanceSegmentToSegmentSq(Vector p1, Vector q1, Vector p2, Vector q2) {
	Vector d1 = Subtract(q1, p1); Vector d2 = Subtract(q2, p2); Vector r = Subtract(p1, p2);
	float a = Dot(d1, d1); float e = Dot(d2, d2); float f = Dot(d2, r);
	float s = 0.0f, t = 0.0f;
	if (a <= 1e-6f && e <= 1e-6f) {}
	else if (a <= 1e-6f) { t = Clamp(f / e, 0.0f, 1.0f); }
	else {
		float c = Dot(d1, r);
		if (e <= 1e-6f) { s = Clamp(-c / a, 0.0f, 1.0f); }
		else {
			float b = Dot(d1, d2); float denom = a * e - b * b;
			s = (fabsf(denom) > 1e-6f) ? Clamp((b * f - c * e) / denom, 0.0f, 1.0f) : 0.0f;
			t = (b * s + f) / e;
			if (t < 0.0f) { t = 0.0f; s = Clamp(-c / a, 0.0f, 1.0f); }
			else if (t > 1.0f) { t = 1.0f; s = Clamp((b - c) / a, 0.0f, 1.0f); }
		}
	}
	return LengthSq(Subtract(Add(p1, Multiply(s, d1)), Add(p2, Multiply(t, d2))));
}

Vector EvaluateBezier(const ArmBezier& arm, float t) {
	float u = 1.0f - t;
	return {
		u * u * arm.shoulder.position.x + 2.0f * u * t * arm.elbow.position.x + t * t * arm.hand.position.x,
		u * u * arm.shoulder.position.y + 2.0f * u * t * arm.elbow.position.y + t * t * arm.hand.position.y,
		u * u * arm.shoulder.position.z + 2.0f * u * t * arm.elbow.position.z + t * t * arm.hand.position.z
	};
}

bool IsSelfIntersectionBezier(const ArmBezier& arm, int subdivisions, float thresholdDistance) {
	if (subdivisions < 4) return false;
	float thresholdSq = thresholdDistance * thresholdDistance;
	std::vector<Vector> points(subdivisions + 1);
	for (int i = 0; i <= subdivisions; ++i) points[i] = EvaluateBezier(arm, (float)i / (float)subdivisions);

	for (int i = 0; i < subdivisions; ++i) {
		for (int j = i + 2; j < subdivisions; ++j) {
			if (ClosestDistanceSegmentToSegmentSq(points[i], points[i + 1], points[j], points[j + 1]) <= thresholdSq) return true;
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
		Vector pZStart = Transform({ offset, 0.0f, -kGridHalfWidth }, vpVpMatrix);
		Vector pZEnd = Transform({ offset, 0.0f, kGridHalfWidth }, vpVpMatrix);
		Vector pXStart = Transform({ -kGridHalfWidth, 0.0f, offset }, vpVpMatrix);
		Vector pXEnd = Transform({ kGridHalfWidth, 0.0f, offset }, vpVpMatrix);
		uint32_t color = (offset == 0.0f) ? 0xFFFFFFFF : 0x888888FF;
		Novice::DrawLine((int)pZStart.x, (int)pZStart.y, (int)pZEnd.x, (int)pZEnd.y, color);
		Novice::DrawLine((int)pXStart.x, (int)pXStart.y, (int)pXEnd.x, (int)pXEnd.y, color);
	}
}

// ベジェ曲線の描画
void DrawArmBezier(const ArmBezier& arm, const Matrix4x4& vpVpMatrix, int subdivisions) {
	Vector previousPoint = arm.shoulder.position;
	for (int i = 1; i <= subdivisions; ++i) {
		float t = (float)i / (float)subdivisions;
		Vector currentPoint = EvaluateBezier(arm, t);
		Vector pStart = Transform(previousPoint, vpVpMatrix);
		Vector pEnd = Transform(currentPoint, vpVpMatrix);

		// 自己交差していない通常時は、グラデーションの代わりに緑転換ベースで描画（今回は白~グレー線の代わりにシンプルに描画）
		Novice::DrawLine((int)pStart.x, (int)pStart.y, (int)pEnd.x, (int)pEnd.y, 0xAAAAAAFF);
		previousPoint = currentPoint;
	}

	// 【関節ノードの描画】階層構造の各点を指定色でスクリーンに描画
	Vector s2d = Transform(arm.shoulder.position, vpVpMatrix);
	Vector e2d = Transform(arm.elbow.position, vpVpMatrix);
	Vector h2d = Transform(arm.hand.position, vpVpMatrix);

	Novice::DrawBox((int)s2d.x - 6, (int)s2d.y - 6, 12, 12, 0.0f, arm.shoulder.color, kFillModeSolid); // 肩: 赤
	Novice::DrawBox((int)e2d.x - 6, (int)e2d.y - 6, 12, 12, 0.0f, arm.elbow.color, kFillModeSolid);    // 肘: 緑
	Novice::DrawBox((int)h2d.x - 6, (int)h2d.y - 6, 12, 12, 0.0f, arm.hand.color, kFillModeSolid);     // 手: 青
}

// --- メイン関数 ---
const char kWindowTitle[] = "3次元腕階層構造（肩:赤 / 肘:緑 / 手:青）";

int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {
	const int kScreenWidth = 1280; const int kScreenHeight = 720;
	Novice::Initialize(kWindowTitle, kScreenWidth, kScreenHeight);

	Vector cameraScale = { 1.0f, 1.0f, 1.0f }, cameraRotate = { 0.785f, 0.0f, 0.0f }, cameraTranslate = { 0.0f, 3.5f, -4.5f };

	// 【初期化】肩＝赤、肘＝緑、手＝青 のカラーコードを設定
	ArmBezier arm = {
		{ { -0.6f,  0.0f,  0.0f }, 0xFF0000FF }, // 肩 (Shoulder) -> 赤
		{ {  0.6f,  1.0f,  0.2f }, 0x00FF00FF }, // 肘 (Elbow)    -> 緑
		{ { -0.6f,  1.0f, -0.2f }, 0x0000FFFF }  // 手 (Hand)     -> 青
	};

	int subdivisions = 48;
	float collisionThreshold = 0.05f;
	char keys[256] = { 0 }, preKeys[256] = { 0 };
	int prevMouseX, prevMouseY;
	Novice::GetMousePosition(&prevMouseX, &prevMouseY);

	while (Novice::ProcessMessage() == 0) {
		Novice::BeginFrame();
		memcpy(preKeys, keys, 256); Novice::GetHitKeyStateAll(keys);

		// マウスカメラ操作
		int mouseX, mouseY; Novice::GetMousePosition(&mouseX, &mouseY);
		int mouseDx = mouseX - prevMouseX, mouseDy = mouseY - prevMouseY, wheel = Novice::GetWheel();
		if (!ImGui::GetIO().WantCaptureMouse) {
			if (Novice::IsPressMouse(0)) { cameraRotate.x += (float)mouseDy * 0.005f; cameraRotate.y += (float)mouseDx * 0.005f; }
			if (Novice::IsPressMouse(2)) {
				float s = 0.01f * fabsf(cameraTranslate.z);
				cameraTranslate.x -= (float)mouseDx * s * cosf(cameraRotate.y) + (float)mouseDy * s * sinf(cameraRotate.y) * sinf(cameraRotate.x);
				cameraTranslate.y += (float)mouseDy * s * cosf(cameraRotate.x);
				cameraTranslate.z -= (float)mouseDx * s * sinf(cameraRotate.y) - (float)mouseDy * s * cosf(cameraRotate.y) * sinf(cameraRotate.x);
			}
			if (wheel != 0) cameraTranslate.z += (float)wheel * 0.1f;
		}
		prevMouseX = mouseX; prevMouseY = mouseY;

		// ImGui 
		ImGui::Begin("Arm Joints Control");
		ImGui::SetWindowSize(ImVec2(350, 150), ImGuiCond_Once);
		ImGui::DragFloat3("Shoulder (赤)", &arm.shoulder.position.x, 0.01f);
		ImGui::DragFloat3("Elbow    (緑)", &arm.elbow.position.x, 0.01f);
		ImGui::DragFloat3("Hand     (青)", &arm.hand.position.x, 0.01f);
		ImGui::End();

		// 自己交差判定（衝突時はノード色を上書き変更するなどの拡張も可能）
		bool isSelfColliding = IsSelfIntersectionBezier(arm, subdivisions, collisionThreshold);

		// 行列計算
		Matrix4x4 viewMatrix = Inverse(MakeAffineMatrix(cameraScale, cameraRotate, cameraTranslate));
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, (float)kScreenWidth / (float)kScreenHeight, 0.1f, 100.0f);
		Matrix4x4 viewportMatrix = MakeViewportMatrix(0.0f, 0.0f, (float)kScreenWidth, (float)kScreenHeight, 0.0f, 1.0f);
		Matrix4x4 vpVpMatrix = Multiply(viewMatrix, Multiply(projectionMatrix, viewportMatrix));

		// 描画
		DrawGrid(vpVpMatrix);

		// 自己交差が検知された場合、手（青）ノードの色を一瞬警告色（明滅やブレンド等）に変える処理を追加するとさらにわかりやすくなります。
		if (isSelfColliding) {
			// 衝突時のデバッグ表示など
			Novice::ScreenPrintf(10, 10, "Status: Self Intersection Detected!");
		}

		DrawArmBezier(arm, vpVpMatrix, subdivisions);

		Novice::EndFrame();
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) break;
	}

	Novice::Finalize();
	return 0;
}