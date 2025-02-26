/*
 * Copyright 2021 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.google.ar.core.examples.kotlin.helloar

import android.annotation.SuppressLint
import android.content.Context
import android.content.res.Resources
import android.opengl.GLSurfaceView
import android.text.InputType
import android.view.View
import android.widget.Button
import android.widget.EditText
import android.widget.ImageButton
import android.widget.LinearLayout
import android.widget.PopupMenu
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AlertDialog
import androidx.lifecycle.DefaultLifecycleObserver
import androidx.lifecycle.LifecycleOwner
import androidx.lifecycle.ViewModelProvider
import com.google.ar.core.Config
import com.google.ar.core.Pose
import com.google.ar.core.examples.java.common.helpers.SnackbarHelper
import com.google.ar.core.examples.java.common.helpers.TapHelper
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.sqrt

/** Contains UI elements for Hello AR. */
class HelloArView(val activity: HelloArActivity) : DefaultLifecycleObserver {

  val root = View.inflate(activity, R.layout.activity_main, null)
  var isOrigin = false
  var isSending = false
  private var sendmsg = DoubleArray(7)
  private var TranslationFromOrigin = FloatArray(16)

  private lateinit var sharedViewModel: SharedViewModel
  init {
    setupViewModel()
  }

  val surfaceView = root.findViewById<GLSurfaceView>(R.id.surfaceview)

  val settingsButton =
    root.findViewById<ImageButton>(R.id.settings_button).apply {
      setOnClickListener { v ->
        PopupMenu(activity, v).apply {
          setOnMenuItemClickListener { item ->
            when (item.itemId) {
              R.id.depth_settings -> launchDepthSettingsMenuDialog()
              R.id.instant_placement_settings -> launchInstantPlacementSettingsMenuDialog()
              R.id.ip_settings -> showIpConfigDialog()
              else -> null
            } != null
          }
          inflate(R.menu.settings_menu)
          show()
        }
      }
    }

  val poseTextView = root.findViewById<TextView>(R.id.poseTextView)

  @SuppressLint("SetTextI18n")
  val startButton = root.findViewById<Button>(R.id.startButton).apply {
    setOnClickListener {
      if((it as Button).text == "开始操作") {
        it.text = "停止操作"
        isOrigin = false
        isSending = true
      } else {
        it.text = "开始操作"
        isSending = false
        poseTextView.text = "六维数据将在这里显示"
      }
    }
  }

  private fun setupViewModel() {
    sharedViewModel = ViewModelProvider(activity)[SharedViewModel::class.java]

    sharedViewModel.viewMatrix.observe(activity as LifecycleOwner) { matrix ->
      matrix?.let {
        updatePoseText(it)
        if (isSending) {
//          poseTextView.text = "sending"
          sendmsg[6] = 1.0
          sharedViewModel.sendFloat64MultiArray(sendmsg)
        } else {
          sendmsg[6] = 0.0
          sharedViewModel.sendFloat64MultiArray(sendmsg)
        }
      }
    }
  }

  private fun updatePoseText(matrix: FloatArray) {
    val formattedText = formatMatrix(matrix)
    poseTextView.text = formattedText
  }

  private fun reverseTranslation(b: FloatArray): FloatArray {
    require(b.size == 16) { "Invalid translation matrices" }
    return floatArrayOf(
      b[0],  // [0]
      b[4],  // [1]
      b[8],  // [2]
      - b[0] * b[3] - b[4] * b[7] - b[8] * b[11],  // [3]
      b[1],  // [4]
      b[5],  // [5]
      b[9],  // [6]
      - b[1] * b[3] - b[5] * b[7] - b[9] * b[11],  // [7]
      b[2],  // [8]
      b[6],  // [9]
      b[10], // [10]
      - b[2] * b[3] - b[6] * b[7] - b[10] * b[11], // [11]
      b[12], // [12]
      b[13], // [13]
      b[14], // [14]
      b[15]  // [15]
    )
  }

  private fun multiply4x4(a: FloatArray, b:FloatArray): FloatArray {
    require(a.size == 16 && b.size == 16) { "Invalid 4x4 matrices" }
    return floatArrayOf(
      a[0]*b[0] + a[1]*b[4] + a[2]*b[8] + a[3]*b[12], // [0]
      a[0]*b[1] + a[1]*b[5] + a[2]*b[9] + a[3]*b[13], // [1]
      a[0]*b[2] + a[1]*b[6] + a[2]*b[10] + a[3]*b[14], // [2]
      a[0]*b[3] + a[1]*b[7] + a[2]*b[11] + a[3]*b[15], // [3]
      a[4]*b[0] + a[5]*b[4] + a[6]*b[8] + a[7]*b[12], // [4]
      a[4]*b[1] + a[5]*b[5] + a[6]*b[9] + a[7]*b[13], // [5]
      a[4]*b[2] + a[5]*b[6] + a[6]*b[10] + a[7]*b[14], // [6]
      a[4]*b[3] + a[5]*b[7] + a[6]*b[11] + a[7]*b[15], // [7]
      a[8]*b[0] + a[9]*b[4] + a[10]*b[8] + a[11]*b[12],  // [8]
      a[8]*b[1] + a[9]*b[5] + a[10]*b[9] + a[11]*b[13], // [9]
      a[8]*b[2] + a[9]*b[6] + a[10]*b[10] + a[11]*b[14], // [10]
      a[8]*b[3] + a[9]*b[7] + a[10]*b[11] + a[11]*b[15], // [11]
      a[12]*b[0] + a[13]*b[4] + a[14]*b[8] + a[15]*b[12], // [12]
      a[12]*b[1] + a[13]*b[5] + a[14]*b[9] + a[15]*b[13], // [13]
      a[12]*b[2] + a[13]*b[6] + a[14]*b[10] + a[15]*b[14], // [14]
      a[12]*b[3] + a[13]*b[7] + a[14]*b[11] + a[15]*b[15] // [15]
    )
  }

  private fun getOrientation(a: FloatArray, b: DoubleArray) {
    require(a.size == 16) { "Invalid 4x4 matrices" }

    val m = DoubleArray(16) {
      a[it].toDouble()
    }
    val cosPitch = sqrt(m[0]*m[0] + m[4]*m[4])
    if (cosPitch < 1e-4) {
      b[0] = 0.0
      if (m[8] > 0) {
        b[1] = PI / 2.0
        b[2] = atan2(m[1], m[5])
      } else {
        b[1] =-PI / 2.0
        b[2] =-atan2(m[1], m[5])
      }
    } else {
      b[0] = atan2(m[4], m[0])
      b[1] = atan2(-m[8], sqrt(m[0] * m[0] + m[4] * m[4]))
      b[2] = atan2(m[9], m[10])
    }
  }

  private fun formatMatrix(matrix: FloatArray): String {
    // initiate to meet the standard
    // the viewMatrix means:
    // [(R^T)^T     0
    // (-R^T*P)^T   1]
    // the R^T*P means the origin pos in view coordinate
    // change it to:
    // [R p
    //  0 1]
    val translationMatrix = FloatArray(16) {
      val row = it / 4
      val col = it % 4
      if (row < 3 && col < 3 )
        matrix[row * 4 + col]
      else {
        if (row < 3)
          -(matrix[row * 4 + 0] * matrix[12] + matrix[row * 4 + 1] * matrix[13] + matrix[row * 4 + 2] * matrix[14])
        else
          matrix[row + col * 4]
      }
    }
    // set origin
    if(!isOrigin) {
      isOrigin = true
      TranslationFromOrigin = reverseTranslation(translationMatrix)
    }

    val orientation = DoubleArray(3)
    // form Phone to Robot
    val tr = floatArrayOf(
      0.0f,-1.0f, 0.0f, 0.0f,
      1.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 1.0f, 0.0f,
      0.0f, 0.0f, 0.0f, 1.0f
    )
    val trT = floatArrayOf(
      0.0f, 1.0f, 0.0f, 0.0f,
      -1.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 1.0f, 0.0f,
      0.0f, 0.0f, 0.0f, 1.0f
    )
    val relativeTranslation = multiply4x4(TranslationFromOrigin, translationMatrix)
    val r1matrix = multiply4x4(trT, relativeTranslation)
    val r2matrix = multiply4x4(r1matrix, tr)
    getOrientation(r2matrix, orientation)
    val roll = Math.toDegrees(orientation[2])
    val pitch = Math.toDegrees(orientation[1])
    val yaw = Math.toDegrees(orientation[0])
    sendmsg[0] = r2matrix[3].toDouble()
    sendmsg[1] = r2matrix[7].toDouble()
    sendmsg[2] = r2matrix[11].toDouble()
    sendmsg[3] = orientation[2]
    sendmsg[4] = orientation[1]
    sendmsg[5] = orientation[0]

    return buildString {
      append("View Matrix:")
      for (i in 0 until 4) {
        append("\n")
        for(j in 0 until 4) {
          append("%.3f ".format(translationMatrix[i * 4 + j]))
        }
      }
      append("\n\nPosition (m):")
      append("\nX: %4.3f".format(r2matrix[3]))
      append("  Y: %4.3f".format(r2matrix[7]))
      append("  Z: %4.3f".format(r2matrix[11]))

      append("\n\nEuler Angles (°):")
      append("\nRoll: %4.2f".format(roll))
      append("  Pitch: %4.2f".format(pitch))
      append("  Yaw: %4.2f".format(yaw))
    }
  }

  val session
    get() = activity.arCoreSessionHelper.session

  val snackbarHelper = SnackbarHelper()
  val tapHelper = TapHelper(activity).also { surfaceView.setOnTouchListener(it) }

  override fun onResume(owner: LifecycleOwner) {
    surfaceView.onResume()
  }

  override fun onPause(owner: LifecycleOwner) {
    surfaceView.onPause()
  }

  /**
   * Shows a pop-up dialog on the first tap in HelloARRenderer, determining whether the user wants
   * to enable depth-based occlusion. The result of this dialog can be retrieved with
   * DepthSettings.useDepthForOcclusion().
   */
  fun showOcclusionDialogIfNeeded() {
    val session = session ?: return
    val isDepthSupported = session.isDepthModeSupported(Config.DepthMode.AUTOMATIC)
    if (!activity.depthSettings.shouldShowDepthEnableDialog() || !isDepthSupported) {
      return // Don't need to show dialog.
    }

    // Asks the user whether they want to use depth-based occlusion.
    AlertDialog.Builder(activity)
      .setTitle(R.string.options_title_with_depth)
      .setMessage(R.string.depth_use_explanation)
      .setPositiveButton(R.string.button_text_enable_depth) { _, _ ->
        activity.depthSettings.setUseDepthForOcclusion(true)
      }
      .setNegativeButton(R.string.button_text_disable_depth) { _, _ ->
        activity.depthSettings.setUseDepthForOcclusion(false)
      }
      .show()
  }

  private fun showIpConfigDialog() {
    val context = activity
    val dialogView = LinearLayout(context).apply {
      orientation = LinearLayout.VERTICAL
      setPadding(32,32,32,32) // 需要定义扩展函数

      // IP 输入框
      addView(EditText(context).apply {
        hint = "服务器IP"
        id = R.id.ip_input
      })

      // 端口输入框
      addView(EditText(context).apply {
        hint = "端口号"
        inputType = InputType.TYPE_CLASS_NUMBER
        id = R.id.port_input
      })
    }

    AlertDialog.Builder(context)
      .setTitle("配置ROS服务器连接")
      .setView(dialogView)
      .setPositiveButton("连接") { _, _ ->
        val ipInput = dialogView.findViewById<EditText>(R.id.ip_input).text.toString().trim()
        val portInput = dialogView.findViewById<EditText>(R.id.port_input).text.toString()

        // 调用 ViewModel 初始化 WebSocket
        sharedViewModel.initWebSocket(ipInput, portInput.toInt())
      }
      .setNegativeButton("取消", null)
      .show()
  }

  private fun launchInstantPlacementSettingsMenuDialog() {
    val resources = activity.resources
    val strings = resources.getStringArray(R.array.instant_placement_options_array)
    val checked = booleanArrayOf(activity.instantPlacementSettings.isInstantPlacementEnabled)
    AlertDialog.Builder(activity)
      .setTitle(R.string.options_title_instant_placement)
      .setMultiChoiceItems(strings, checked) { _, which, isChecked -> checked[which] = isChecked }
      .setPositiveButton(R.string.done) { _, _ ->
        val session = session ?: return@setPositiveButton
        activity.instantPlacementSettings.isInstantPlacementEnabled = checked[0]
        activity.configureSession(session)
      }
      .show()
  }

  /** Shows checkboxes to the user to facilitate toggling of depth-based effects. */
  private fun launchDepthSettingsMenuDialog() {
    val session = session ?: return

    // Shows the dialog to the user.
    val resources: Resources = activity.resources
    val checkboxes =
      booleanArrayOf(
        activity.depthSettings.useDepthForOcclusion(),
        activity.depthSettings.depthColorVisualizationEnabled()
      )
    if (session.isDepthModeSupported(Config.DepthMode.AUTOMATIC)) {
      // With depth support, the user can select visualization options.
      val stringArray = resources.getStringArray(R.array.depth_options_array)
      AlertDialog.Builder(activity)
        .setTitle(R.string.options_title_with_depth)
        .setMultiChoiceItems(stringArray, checkboxes) { _, which, isChecked ->
          checkboxes[which] = isChecked
        }
        .setPositiveButton(R.string.done) { _, _ ->
          activity.depthSettings.setUseDepthForOcclusion(checkboxes[0])
          activity.depthSettings.setDepthColorVisualizationEnabled(checkboxes[1])
        }
        .show()
    } else {
      // Without depth support, no settings are available.
      AlertDialog.Builder(activity)
        .setTitle(R.string.options_title_without_depth)
        .setPositiveButton(R.string.done) { _, _ -> /* No settings to apply. */ }
        .show()
    }
  }
}
