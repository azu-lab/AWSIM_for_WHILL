// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Collections.Generic;
using UnityEngine;

namespace Awsim.Optimizer
{
    /// <summary>
    /// メインスレッドでアクションを実行するためのディスパッチャー
    /// Unity APIはメインスレッドからのみアクセス可能なため、
    /// 別スレッドからの操作をキューイングして実行する
    /// </summary>
    public class UnityMainThreadDispatcher : MonoBehaviour
    {
        private static UnityMainThreadDispatcher _instance;
        private readonly Queue<Action> _executionQueue = new Queue<Action>();
        private readonly object _lock = new object();
        
        public static UnityMainThreadDispatcher Instance
        {
            get
            {
                if (_instance == null)
                {
                    var go = new GameObject("UnityMainThreadDispatcher");
                    _instance = go.AddComponent<UnityMainThreadDispatcher>();
                    DontDestroyOnLoad(go);
                }
                return _instance;
            }
        }
        
        void Awake()
        {
            if (_instance == null)
            {
                _instance = this;
                DontDestroyOnLoad(gameObject);
            }
            else if (_instance != this)
            {
                Destroy(gameObject);
            }
        }
        
        void Update()
        {
            lock (_lock)
            {
                while (_executionQueue.Count > 0)
                {
                    var action = _executionQueue.Dequeue();
                    try
                    {
                        action?.Invoke();
                    }
                    catch (Exception e)
                    {
                        Debug.LogError($"[UnityMainThreadDispatcher] Error executing action: {e}");
                    }
                }
            }
        }
        
        /// <summary>
        /// メインスレッドで実行するアクションをキューに追加
        /// </summary>
        public void Enqueue(Action action)
        {
            if (action == null)
                return;
                
            lock (_lock)
            {
                _executionQueue.Enqueue(action);
            }
        }
        
        /// <summary>
        /// キューをクリア
        /// </summary>
        public void Clear()
        {
            lock (_lock)
            {
                _executionQueue.Clear();
            }
        }
    }
}
