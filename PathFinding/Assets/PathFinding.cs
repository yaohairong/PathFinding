using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;

public class PathFinding : MonoBehaviour {
    //宽度
    const int WIDTH = 40;

    //高度
    const int HEIGHT = 30;

    //相邻节点
    class Neighbour {
        public Vector2 offset;
        public int distance;

        public Neighbour(Vector2 offset, int distance) {
            this.offset = offset;
            this.distance = distance;
        }
    }

    //相邻节点列表
    Neighbour[] neighbourList = {
        new Neighbour(
            new Vector2(-1, 0),
            1
        ),
         new Neighbour(
            new Vector2(0, 1),
            1
        ),
         new Neighbour(
            new Vector2(1, 0),
            1
        ),
         new Neighbour(
            new Vector2(0, -1),
            1
        ),
    };

    //搜索类型
    private enum SearchAlgorithm {
        [Description("深度优先搜索")]
        DepthFirst,

        [Description("广度优先搜索")]
        BreadthFirst,

        [Description("地杰斯特拉")]
        Dijkstra,

        [Description("A星")]
        AStar,
    }

    //节点
    public class Node {
        //位置
        public Vector2 pos;

        //距离起点的距离
        public int costG;

        //到终点的估值
        public int costH;

        //阻挡
        public bool block;

        //父节点
        public Node parent;

        //到起点的距离加到终点的估值
        public int costF {
            get {
                return costG + costH;
            }
        }

        //方便调试
        public override string ToString() {
            return string.Format("{0}, {1}", pos.x, pos.y);
        }
    }

    //格子地图
    private Node[,] gridMap = new Node[WIDTH, HEIGHT];

    //开放列表
    private List<Node> openList = new List<Node>();

    //关闭列表
    private List<Node> closeList = new List<Node>();

    //起点
    public Node start;

    //终点
    public Node end;

    //当前处理节点
    public Node current;

    //搜索周期
    public float period = 0.01f;

    //是否正在搜索
    private bool searching;

    //当前搜索算法
    private SearchAlgorithm searchAlgorithm;

    //当前搜索协程
    private Coroutine searchingCoroutine;

    void Awake() {
        //关闭垂直同步
        QualitySettings.vSyncCount = 0;

        //初始化格子地图
        for (int x = 0; x < WIDTH; x++) {
            for (int y = 0; y < HEIGHT; y++) {
                Node node = new Node {
                    pos = new Vector2(x, y),
                };

                gridMap[x, y] = node;
            }
        }

        //初始化起点和终点
        start = gridMap[0, 0];
        start.parent = start;
        end = gridMap[WIDTH - 1, HEIGHT - 1];
    }

    void Start() {

    }

    //开始搜索
    void StartSearch() {
        StopSearch();
        current = null;

        searching = true;
        searchingCoroutine = StartCoroutine(SearchCoroutine());
    }

    //停止搜索
    void StopSearch() {
        searching = false;
        if (searchingCoroutine != null) {
            StopCoroutine(searchingCoroutine);
        }
    }

    //根据坐标取节点
    Node ConvertToNode(Vector3 mousePosition) {
        var screePosition = Camera.main.ScreenToWorldPoint(mousePosition);
        int x = (int)screePosition.x;
        int y = (int)screePosition.y;
        if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
            return gridMap[x, y];
        }
        return null;
    }

    //清空开放列表
    void ClearOpenList() {
        openList.Clear();
    }

    //清空关闭列表
    void ClearCloseList() {
        closeList.Clear();
    }

    //清除所有阻挡
    void ClearBlock() {
        for (int x = 0; x < WIDTH; x++) {
            for (int y = 0; y < HEIGHT; y++) {
                gridMap[x, y].block = false;
            }
        }
    }

    //清屏
    void Clear() {
        current = null;

        ClearOpenList();
        ClearCloseList();
    }

    //重置
    void Reset() {
        start = null;
        end = null;
        current = null;

        ClearBlock();
        ClearOpenList();
        ClearCloseList();
    }

    void Update() {
        //深度优先搜索
        if (Input.GetKeyDown(KeyCode.D)) {
            searchAlgorithm = SearchAlgorithm.DepthFirst;
            StartSearch();
        }
        //广度优先搜索
        else if (Input.GetKeyDown(KeyCode.B)) {
            searchAlgorithm = SearchAlgorithm.BreadthFirst;
            StartSearch();
        }
        //迪杰斯特拉搜索
        else if (Input.GetKeyDown(KeyCode.J)) {
            searchAlgorithm = SearchAlgorithm.Dijkstra;
            StartSearch();
        }
        //A星搜索
        else if (Input.GetKeyDown(KeyCode.A)) {
            searchAlgorithm = SearchAlgorithm.AStar;
            StartSearch();
        }
        //清屏
        else if (Input.GetKeyDown(KeyCode.C)) {
            StopSearch();
            Clear();
        }
        //重置
        else if (Input.GetKeyDown(KeyCode.R)) {
            StopSearch();
            Reset();
        }

        var mousePosition = Input.mousePosition;
        bool mouse0 = Input.GetKeyDown(KeyCode.Mouse0);
        bool mouse1 = Input.GetKeyDown(KeyCode.Mouse1);

        if (!searching) {
            //左键设置起点
            if (mouse0) {
                var node = ConvertToNode(mousePosition);
                if (node != null) {
                    start = node;
                    start.block = false;
                }
            }
            //右键设置终点
            else if (mouse1) {
                var node = ConvertToNode(mousePosition);
                if (node != null) {
                    end = node;
                    end.block = false;
                }
            }

            bool leftControl = Input.GetKey(KeyCode.LeftControl);
            bool leftAlt = Input.GetKey(KeyCode.LeftShift);

            //按照ctrl设置阻挡，按住shift清除阻挡
            if (leftControl || leftAlt) {
                var node = ConvertToNode(mousePosition);
                if (node != null && node != start && node != end) {
                    node.block = leftControl;
                }
            }
        }
    }

    //搜索协程
    IEnumerator SearchCoroutine() {
        Clear();

        //清空父节点
        foreach (var node in gridMap) {
            node.parent = null;
        }

        start.parent = start;
        start.costG = 0;
        start.costH = Mathf.Abs((int)(end.pos.x - end.pos.x)) + Mathf.Abs((int)(end.pos.y - end.pos.x));
        openList.Add(start);

        while (searching) {
            if (openList.Count == 0) {
                Debug.Log("no path");
                break;
            }

            int nextNodeIndex = 0;

            //从开放列表选取下一个节点
            switch (searchAlgorithm) {
                case SearchAlgorithm.DepthFirst:
                    //深度优先搜索后进先出，可以优化为栈(Stack)
                    nextNodeIndex = openList.Count - 1;
                    break;
                case SearchAlgorithm.BreadthFirst:
                    //广度优先搜索先进先出，可以优化为队列
                    nextNodeIndex = 0;
                    break;
                //迪杰斯特拉从开放列表找离起点最近的节点
                case SearchAlgorithm.Dijkstra: {
                        nextNodeIndex = openList.Count - 1;
                        int minCostG = openList[nextNodeIndex].costG;
                        for (int i = openList.Count - 1; i >= 0; i--) {
                            if (openList[i].costG < minCostG) {
                                minCostG = openList[i].costG;
                                nextNodeIndex = i;
                            }
                        }
                    }
                    break;
                //A星从开放列表找离起点距离加离终点估值最小的节点
                case SearchAlgorithm.AStar: {
                        nextNodeIndex = openList.Count - 1;
                        int minCostF = openList[nextNodeIndex].costF;
                        for (int i = openList.Count - 1; i >= 0; i--) {
                            if (openList[i].costF < minCostF) {
                                minCostF = openList[i].costF;
                                nextNodeIndex = i;
                            }
                        }
                    }
                    break;
                default:
                    break;
            }

            current = openList[nextNodeIndex];

            //找到路径
            if (current == end) {
                break;
            }

            //把当前节点从开放列表删除，加入关闭列表
            openList.Remove(current);
            closeList.Add(current);

            //对当前节点邻居展开加入开放列表
            for (int i = 0; i < neighbourList.Length; i++) {
                var offset = neighbourList[i].offset;
                int x = (int)(current.pos.x + offset.x);
                int y = (int)(current.pos.y + offset.y);

                if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
                    var node = gridMap[x, y];
                    if (!node.block && !closeList.Contains(node)) {
                        if (!openList.Contains(node)) {
                            node.parent = current;
                            node.costG = current.costG + neighbourList[i].distance;
                            node.costH = Mathf.Abs((int)(end.pos.x - node.pos.x)) + Mathf.Abs((int)(end.pos.y - node.pos.y));
                            switch (searchAlgorithm) {
                                case SearchAlgorithm.DepthFirst: {
                                        //深度优先搜索后进先出，可以优化为栈(Stack)
                                        openList.Add(node);
                                    }
                                    break;
                                case SearchAlgorithm.Dijkstra:
                                case SearchAlgorithm.AStar:
                                case SearchAlgorithm.BreadthFirst: {
                                        //广度优先搜索先进先出，可以优化为队列
                                        openList.Add(node);
                                    }
                                    break;
                                default:
                                    break;
                            }
                        }
                        else {
                            //迪杰斯特拉和A星遇到已在开放列表中的邻居节点，但从此处到达G值更低的，重新计算该邻居节点的G值并调整该节点的父节点为当前节点
                            if (searchAlgorithm == SearchAlgorithm.DepthFirst || searchAlgorithm == SearchAlgorithm.AStar) {
                                int costG = current.costG + neighbourList[i].distance;
                                if (costG < node.costG) {
                                    node.costG = costG;
                                    node.parent = current;
                                }
                            }
                        }
                    }
                }
            }

            if (period > 0) {
                yield return new WaitForSeconds(period);
            }
        }
    }

    //画矩形
    void DrawRectangle(float x, float y, Color color) {
        DrawRectangle(x, y, color, 0.5f);
    }

    //画矩形
    void DrawRectangle(float x, float y, Color color, float size) {
        Gizmos.color = color;
        Vector3 center = new Vector3(x, y);
        Vector3 leftTop = center + new Vector3(-size, size);
        Vector3 rightTop = center + new Vector3(size, size);
        Vector3 leftBottom = center + new Vector3(-size, -size);
        Vector3 rightBottom = center + new Vector3(size, -size);

        Gizmos.DrawLine(leftTop, rightTop);
        Gizmos.DrawLine(rightTop, rightBottom);
        Gizmos.DrawLine(rightBottom, leftBottom);
        Gizmos.DrawLine(leftBottom, leftTop);
    }

    //画球体
    void DrawSphere(float x, float y, Color color) {
        Gizmos.color = color;
        Gizmos.DrawSphere(new Vector3(x, y), 0.5f);
    }

    //绘制格子地图
    void DrawGridMap() {
        for (int x = 0; x < WIDTH; x++) {
            for (int y = 0; y < HEIGHT; y++) {
                Node node = gridMap[x, y];
                if (node != null) {
                    if (node.block) {
                        DrawSphere(node.pos.x, node.pos.y, Color.black);
                    }
                    else {
                        DrawRectangle(node.pos.x, node.pos.y, Color.black);
                    }
                }
            }
        }
    }

    //绘制开放列表
    void DrawOpenList() {
        foreach (var node in openList) {
            DrawRectangle(node.pos.x, node.pos.y, Color.green);
        }
    }

    //绘制关闭列表
    void DrawCloseList() {
        foreach (var node in closeList) {
            DrawSphere(node.pos.x, node.pos.y, Color.gray);
        }
    }

    //绘制起点
    void DrawStart() {
        if (start != null) {
            DrawSphere(start.pos.x, start.pos.y, Color.green);
        }
    }

    //绘制终点
    void DrawEnd() {
        if (end != null) {
            DrawSphere(end.pos.x, end.pos.y, Color.red);
        }
    }

    //绘制当前点
    void DrawCurrent() {
        if (current != null) {
            DrawSphere(current.pos.x, current.pos.y, Color.yellow);
        }
    }

    //绘制路径
    void DrawPath() {
        Gizmos.color = Color.green;
        Node lastNode = current;
        while (lastNode != null && lastNode != start) {
            var node1 = lastNode;
            var node2 = lastNode.parent;
            Gizmos.DrawLine(new Vector3(node1.pos.x, node1.pos.y), new Vector3(node2.pos.x, node2.pos.y));
            lastNode = lastNode.parent;
        }
    }

    void OnDrawGizmos() {
        DrawGridMap();

        DrawOpenList();

        DrawCloseList();

        DrawStart();

        DrawEnd();

        DrawCurrent();

        DrawPath();
    }

    void OnGUI() {
        GUI.Label(new Rect(new Vector2(10, 10), new Vector2(300, 500)), "寻路算法演示(需在Game窗口打开Gizmos)\n左键设置起点\n右键设置终点\nC清屏\nR重置\n按住Ctrl拖动鼠标设置阻挡\n按住Shift拖动鼠标清除阻挡\nD开始深度优先搜索\nB开始广度优先搜索\nD开始迪杰斯特拉搜索\nA开始A*搜索\n");
    }
}
