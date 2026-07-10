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

struct Node {
	Vector scale;
	Vector rotate;
	Vector translate;
	Matrix4x4 localMatrix;
	Matrix4x4 worldMatrix;
	uint32_t color;
};

struct ArmHierarchy {
	Node shoulder; // 親 (Shoulder) [赤]
	Node elbow;    // 子 (Elbow)    [緑]
	Node hand;     // 孫 (Hand)     [青]
};

// --- 💡 演算子オーバーロード (Operator Overloading) ---

// ベクトル + ベクトル
Vector operator+(const Vector& v1, const Vector& v2) {
	return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
}

// ベクトル - ベクトル
Vector operator-(const Vector& v1, const Vector& v2) {
	return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
}

// スカラー * ベクトル
Vector operator*(float k, const Vector& v) {
	return { k * v.x, k * v.y, k * v.z };
}

// ベクトル * スカラー
Vector operator*(const Vector& v, float k) {
	return { v.x * k, v.y * k, v.z * k };
}

// 行列 * 行列
Matrix4x4 operator*(const Matrix4x4& A, const Matrix4x4& B) {
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			for (int k = 0; k < 4; k++) {
				result.m[i][j] += A.m[i][k] * B.m[k][j];
			}
		}
	}
	return result;
}


// --- 行列・変換演算関数 ---

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

	// 💡 演算子オーバーロードでスッキリ結合
	Matrix4x4 rotMat = rotateX * rotateY * rotateZ;

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

Vector GetWorldPosition(const Matrix4x4& mat) {
	return { mat.m[3][0], mat.m[3][1], mat.m[3][2] };
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

void DrawArmHierarchy(const ArmHierarchy& arm, const Matrix4x4& vpVpMatrix) {
	Vector shoulderWorld = GetWorldPosition(arm.shoulder.worldMatrix);
	Vector elbowWorld = GetWorldPosition(arm.elbow.worldMatrix);
	Vector handWorld = GetWorldPosition(arm.hand.worldMatrix);

	Vector s2d = Transform(shoulderWorld, vpVpMatrix);
	Vector e2d = Transform(elbowWorld, vpVpMatrix);
	Vector h2d = Transform(handWorld, vpVpMatrix);

	Novice::DrawLine((int)s2d.x, (int)s2d.y, (int)e2d.x, (int)e2d.y, 0xAAAAAAFF);
	Novice::DrawLine((int)e2d.x, (int)e2d.y, (int)h2d.x, (int)h2d.y, 0xAAAAAAFF);

	Novice::DrawBox((int)s2d.x - 6, (int)s2d.y - 6, 12, 12, 0.0f, arm.shoulder.color, kFillModeSolid);
	Novice::DrawBox((int)e2d.x - 6, (int)e2d.y - 6, 12, 12, 0.0f, arm.elbow.color, kFillModeSolid);
	Novice::DrawBox((int)h2d.x - 6, (int)h2d.y - 6, 12, 12, 0.0f, arm.hand.color, kFillModeSolid);
}

// --- メイン関数 ---
const char kWindowTitle[] = "3次元腕階層構造（肩:赤 / 肘:緑 / 手:青）";

int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {
	const int kScreenWidth = 1280; const int kScreenHeight = 720;
	Novice::Initialize(kWindowTitle, kScreenWidth, kScreenHeight);

	Vector cameraScale = { 1.0f, 1.0f, 1.0f }, cameraRotate = { 0.785f, 0.0f, 0.0f }, cameraTranslate = { 0.0f, 3.5f, -6.5f };

	ArmHierarchy arm;

	// 肩 (Shoulder)
	arm.shoulder.scale = { 1.0f, 1.0f, 1.0f };
	arm.shoulder.rotate = { 0.0f, 0.0f, 0.0f };
	arm.shoulder.translate = { -1.0f, 0.0f, 0.0f };
	arm.shoulder.color = 0xFF0000FF;

	// 肘 (Elbow)
	arm.elbow.scale = { 1.0f, 1.0f, 1.0f };
	arm.elbow.rotate = { 0.0f, 0.0f, 0.0f };
	arm.elbow.translate = { 1.0f, 0.5f, 0.0f };
	arm.elbow.color = 0x00FF00FF;

	// 手 (Hand)
	arm.hand.scale = { 1.0f, 1.0f, 1.0f };
	arm.hand.rotate = { 0.0f, 0.0f, 0.0f };
	arm.hand.translate = { 1.0f, -0.5f, 0.0f };
	arm.hand.color = 0x0000FFFF;

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

		// ImGui描画
		ImGui::Begin("Arm Hierarchy Control");
		ImGui::SetWindowSize(ImVec2(400, 320), ImGuiCond_Once);

		if (ImGui::TreeNode("Shoulder (肩:親) [赤]")) {
			ImGui::DragFloat3("Translate", &arm.shoulder.translate.x, 0.01f);
			ImGui::SliderFloat3("Rotate", &arm.shoulder.rotate.x, -3.14f, 3.14f);
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Elbow (肘:子) [緑]")) {
			ImGui::DragFloat3("Translate (相対)", &arm.elbow.translate.x, 0.01f);
			ImGui::SliderFloat3("Rotate", &arm.elbow.rotate.x, -3.14f, 3.14f);
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Hand (手:孫) [青]")) {
			ImGui::DragFloat3("Translate (相対)", &arm.hand.translate.x, 0.01f);
			ImGui::SliderFloat3("Rotate", &arm.hand.rotate.x, -3.14f, 3.14f);
			ImGui::TreePop();
		}
		ImGui::End();

		// 1. 各ノードのローカル行列を生成
		arm.shoulder.localMatrix = MakeAffineMatrix(arm.shoulder.scale, arm.shoulder.rotate, arm.shoulder.translate);
		arm.elbow.localMatrix = MakeAffineMatrix(arm.elbow.scale, arm.elbow.rotate, arm.elbow.translate);
		arm.hand.localMatrix = MakeAffineMatrix(arm.hand.scale, arm.hand.rotate, arm.hand.translate);

		// 2. 💡 階層構造の行列計算を演算子（*）でスッキリ記述
		arm.shoulder.worldMatrix = arm.shoulder.localMatrix;
		arm.elbow.worldMatrix = arm.elbow.localMatrix * arm.shoulder.worldMatrix;
		arm.hand.worldMatrix = arm.hand.localMatrix * arm.elbow.worldMatrix;

		// 3. 💡 カメラ・ビューポート行列計算も演算子（*）で直感的に合成
		Matrix4x4 viewMatrix = Inverse(MakeAffineMatrix(cameraScale, cameraRotate, cameraTranslate));
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, (float)kScreenWidth / (float)kScreenHeight, 0.1f, 100.0f);
		Matrix4x4 viewportMatrix = MakeViewportMatrix(0.0f, 0.0f, (float)kScreenWidth, (float)kScreenHeight, 0.0f, 1.0f);

		Matrix4x4 vpVpMatrix = viewMatrix * projectionMatrix * viewportMatrix;

		// 描画
		DrawGrid(vpVpMatrix);
		DrawArmHierarchy(arm, vpVpMatrix);

		Novice::EndFrame();
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) break;
	}

	Novice::Finalize();
	return 0;
}