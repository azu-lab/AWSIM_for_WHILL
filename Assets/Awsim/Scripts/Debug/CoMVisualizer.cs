using UnityEngine;

/// <summary>
/// 重心（Center of Mass）を可視化するデバッグスクリプト
/// Rigidbodyがアタッチされたオブジェクト（WhillModelCRなど）に追加してください
/// </summary>
public class CoMVisualizer : MonoBehaviour
{
    [Header("表示設定")]
    [SerializeField] private float _sphereRadius = 0.08f;
    [SerializeField] private float _axisLength = 0.5f;
    [SerializeField] private bool _showInPlayMode = true;
    [SerializeField] private bool _showInEditMode = true;
    
    [Header("色設定")]
    [SerializeField] private Color _comColor = Color.magenta;
    [SerializeField] private Color _forwardColor = Color.blue;
    [SerializeField] private Color _upColor = Color.green;
    [SerializeField] private Color _rightColor = Color.red;
    
    private Rigidbody _rb;
    
    void Awake()
    {
        _rb = GetComponent<Rigidbody>();
    }
    
    void OnDrawGizmos()
    {
        // Edit Mode での表示制御
        if (!Application.isPlaying && !_showInEditMode) return;
        // Play Mode での表示制御
        if (Application.isPlaying && !_showInPlayMode) return;
        
        if (_rb == null) _rb = GetComponent<Rigidbody>();
        if (_rb == null) return;
        
        // ローカル座標からワールド座標に変換
        Vector3 comWorld = transform.TransformPoint(_rb.centerOfMass);
        
        // CoMを球で表示
        Gizmos.color = _comColor;
        Gizmos.DrawSphere(comWorld, _sphereRadius);
        Gizmos.DrawWireSphere(comWorld, _sphereRadius * 1.5f);
        
        // 軸表示（車両の向きを確認）
        // Z軸 = 前方（青）
        Gizmos.color = _forwardColor;
        Gizmos.DrawLine(comWorld, comWorld + transform.forward * _axisLength);
        DrawArrowHead(comWorld + transform.forward * _axisLength, transform.forward);
        
        // Y軸 = 上方（緑）
        Gizmos.color = _upColor;
        Gizmos.DrawLine(comWorld, comWorld + transform.up * _axisLength * 0.6f);
        
        // X軸 = 右方（赤）
        Gizmos.color = _rightColor;
        Gizmos.DrawLine(comWorld, comWorld + transform.right * _axisLength * 0.4f);
    }
    
    private void DrawArrowHead(Vector3 position, Vector3 direction)
    {
        float arrowSize = 0.1f;
        Vector3 right = Vector3.Cross(direction, Vector3.up).normalized;
        Vector3 up = Vector3.Cross(right, direction).normalized;
        
        Gizmos.DrawLine(position, position - direction * arrowSize + right * arrowSize * 0.5f);
        Gizmos.DrawLine(position, position - direction * arrowSize - right * arrowSize * 0.5f);
    }
    
    // ランタイムでのCoM情報をコンソールに出力
    [ContextMenu("Log CoM Info")]
    public void LogCoMInfo()
    {
        if (_rb == null) _rb = GetComponent<Rigidbody>();
        if (_rb == null)
        {
            Debug.LogWarning("Rigidbody not found!");
            return;
        }
        
        Vector3 comLocal = _rb.centerOfMass;
        Vector3 comWorld = transform.TransformPoint(comLocal);
        
        Debug.Log($"=== Center of Mass Info ===");
        Debug.Log($"Local CoM: ({comLocal.x:F3}, {comLocal.y:F3}, {comLocal.z:F3})");
        Debug.Log($"World CoM: ({comWorld.x:F3}, {comWorld.y:F3}, {comWorld.z:F3})");
        Debug.Log($"Vehicle Forward: {transform.forward}");
        Debug.Log($"Vehicle Position: {transform.position}");
    }
}
