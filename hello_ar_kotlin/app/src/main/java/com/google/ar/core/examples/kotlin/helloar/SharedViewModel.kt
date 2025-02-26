package com.google.ar.core.examples.kotlin.helloar

import android.app.Application
import android.content.Context
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import android.util.Log
import com.google.gson.Gson
import com.google.gson.JsonObject
import com.google.gson.JsonArray
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.java_websocket.client.WebSocketClient
import org.java_websocket.handshake.ServerHandshake
import org.jetbrains.annotations.Async.Execute
import java.net.URI
import java.nio.ByteBuffer

class SharedViewModel : ViewModel() {
    private val _viewMatrix = MutableLiveData<FloatArray>()
    val viewMatrix: LiveData<FloatArray> = _viewMatrix

    private var webSocketClient: WebSocketClient? = null
    private var currentIP: String = "192.168.97.173"
    private var currentPort: Int = 9091

    private val gson = Gson()

//    init {
//        initWebSocket()
//    }

    fun updateMatrix(matrix: FloatArray) {
        _viewMatrix.postValue(matrix.copyOf())
    }

    fun initWebSocket(IP: String, Port: Int) {
        webSocketClient?.close()
        webSocketClient = null

        currentIP = IP
        currentPort = Port

        viewModelScope.launch(Dispatchers.IO) {
            try {
                val uri = URI("ws://$currentIP:$currentPort")
                webSocketClient = object : WebSocketClient(uri) {
                    override fun onOpen(handshakedata: ServerHandshake?) {
                        Log.d("ROS", "Connected to rosbridge")
                    }

                    override fun onClose(code: Int, reason: String?, remote: Boolean) {
                        Log.d("ROS", "Connection closed")
                    }

                    override fun onMessage(message: String?) {
                        Log.d("Websocket", "Received: $message")
                    }
                    override fun onError(ex: java.lang.Exception?) {
                        Log.e("ROS", "WebSocket error",ex)
                    }
                }
                withContext(Dispatchers.Main) {
                    webSocketClient?.connect()
                }
            } catch (e: Exception) {
                Log.e("ROS", "WebSocket init failed", e)
            }
        }
    }

    fun sendFloat64MultiArray(data: DoubleArray) {
        val jsonMessage = buildFloat64MultiArrayMessage(data)
        viewModelScope.launch(Dispatchers.IO) {
            try {
                webSocketClient?.send(jsonMessage)
            } catch (e: Exception) {
                Log.e("ROS", "Send failed", e)
            }
        }
    }

    private fun buildFloat64MultiArrayMessage(data: DoubleArray): String {
        val json = JsonObject().apply {
            addProperty("op", "publish")
            addProperty("topic", "/ar_pose")
            add("msg", JsonObject().apply {
                add("layout", JsonObject().apply {
                    add("dim", JsonArray().apply {
                        add(JsonObject().apply {
                            addProperty("label", "")
                            addProperty("size",data.size)
                            addProperty("stride", 1)
                        })
                    })
                })
                add("data", JsonArray().apply {
                    data.forEach { add(it) }
                })
            })
        }
        return gson.toJson(json)
    }

    override fun onCleared() {
        webSocketClient?.close()
        super.onCleared()
    }
}