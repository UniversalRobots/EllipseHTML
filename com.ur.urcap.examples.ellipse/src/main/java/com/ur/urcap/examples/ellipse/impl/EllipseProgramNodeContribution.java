package com.ur.urcap.examples.ellipse.impl;

import com.ur.urcap.api.contribution.ProgramNodeContribution;
import com.ur.urcap.api.domain.URCapAPI;
import com.ur.urcap.api.domain.data.DataModel;
import com.ur.urcap.api.domain.feature.Feature;
import com.ur.urcap.api.domain.program.ProgramModel;
import com.ur.urcap.api.domain.program.nodes.ProgramNodeFactory;
import com.ur.urcap.api.domain.program.nodes.builtin.MoveNode;
import com.ur.urcap.api.domain.program.nodes.builtin.WaypointNode;
import com.ur.urcap.api.domain.program.nodes.builtin.configurations.movenode.MovePMotionParameters;
import com.ur.urcap.api.domain.program.nodes.builtin.configurations.waypointnode.BlendParameters;
import com.ur.urcap.api.domain.program.nodes.builtin.configurations.waypointnode.WaypointMotionParameters;
import com.ur.urcap.api.domain.program.nodes.builtin.configurations.waypointnode.WaypointNodeConfig;
import com.ur.urcap.api.domain.program.nodes.builtin.configurations.waypointnode.WaypointNodeConfigFactory;
import com.ur.urcap.api.domain.program.structure.TreeNode;
import com.ur.urcap.api.domain.program.structure.TreeStructureException;
import com.ur.urcap.api.domain.script.ScriptWriter;
import com.ur.urcap.api.domain.userinteraction.RobotPositionCallback;
import com.ur.urcap.api.domain.validation.ErrorHandler;
import com.ur.urcap.api.domain.value.Pose;
import com.ur.urcap.api.domain.value.blend.Blend;
import com.ur.urcap.api.domain.value.jointposition.JointPositions;
import com.ur.urcap.api.domain.value.simple.Acceleration;
import com.ur.urcap.api.domain.value.simple.Angle;
import com.ur.urcap.api.domain.value.simple.Length;
import com.ur.urcap.api.domain.value.simple.SimpleValueFactory;
import com.ur.urcap.api.domain.value.simple.Speed;
import com.ur.urcap.api.ui.annotation.Input;
import com.ur.urcap.api.ui.annotation.Label;
import com.ur.urcap.api.ui.component.InputEvent;
import com.ur.urcap.api.ui.component.LabelComponent;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class EllipseProgramNodeContribution implements ProgramNodeContribution {

	private static final String ERROR_LABEL = "errorLabel";
	private static final String DEFINED_KEY = "is_defined";
	private static final String BUTTON_SELECT_POSE = "buttonSelectPose";

	private static final double SHARED_TOOL_SPEED = 250;
	private static final double SHARED_TOOL_ACCELERATION = 1200;
	private static final double SHARED_BLEND_RADIUS_IN_MM = 23;
	private static final double HORIZONTAL_RADIUS_IN_MM = 200.0;
	private static final double VERTICAL_RADIUS_IN_MM = 120.0;

	private static final int NUMBER_OF_WAYPOINTS = 16;

	private final URCapAPI urCapAPI;

	private MoveNode moveNode;
	private final List<WaypointNode> waypointNodes = new ArrayList<WaypointNode>();

	private DataModel dataModel;
	private TreeNode moveTreeNode;

	private WaypointNodeConfigFactory waypointNodeConfigFactory;
	private ProgramNodeFactory programNodeFactory;

	private final BufferedImage errorIcon;

	@Label(id = ERROR_LABEL)
	public LabelComponent errorLabel;

	public EllipseProgramNodeContribution(URCapAPI urCapAPI, DataModel dataModel) {
		this.urCapAPI = urCapAPI;
		this.dataModel = dataModel;
		this.errorIcon = getErrorImage();

		programNodeFactory = urCapAPI.getProgramModel().getProgramNodeFactory();
		waypointNodeConfigFactory = programNodeFactory.createWaypointNode().getConfigFactory();
	}

	@Input(id = BUTTON_SELECT_POSE)
	public void onSelectPoseButtonPressed(InputEvent event) {
		if (event.getEventType() == InputEvent.EventType.ON_PRESSED) {
			selectCenterPoint();
		}
	}

	private void selectCenterPoint() {
		urCapAPI.getUserInteraction().getUserDefinedRobotPosition(new RobotPositionCallback() {
			@Override
			public void onOk(Pose pose, JointPositions jointPositions) {
				removeNodes();
				createNodes();
				configureMoveNode();
				adjustWaypointsToCenterPoint(pose, jointPositions);
			}
		});
	}

	private void removeNodes() {
		TreeNode rootTreeNode = urCapAPI.getProgramModel().getRootTreeNode(this);
		try {
			Iterator<TreeNode> it = rootTreeNode.getChildren().iterator();
			while (it.hasNext()) {
				TreeNode treeNode = it.next();
				rootTreeNode.removeChild(treeNode);
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private void adjustWaypointsToCenterPoint(Pose startPose, JointPositions jointPositions) {
		try {
			configureWaypointNodes(startPose, jointPositions);
			clearErrors();
		} catch (IllegalArgumentException e) {
			setError();
			resetWaypointNodes();
		}
		lockTreeNodes();
	}

	private void resetWaypointNodes() {
		BlendParameters blendParameters = waypointNodeConfigFactory.createSharedBlendParameters();
		WaypointMotionParameters motionParameters = waypointNodeConfigFactory.createSharedMotionParameters();
		for (WaypointNode waypointNode : waypointNodes) {
			WaypointNodeConfig waypointNodeConfig = waypointNodeConfigFactory.
					createFixedPositionConfig(blendParameters, motionParameters);
			waypointNode.setConfig(waypointNodeConfig);
		}
	}

	private void configureWaypointNodes(Pose centerPose, JointPositions jointPositions) {
		// adjust orientation according to base
		double baseAngle = jointPositions.getAllJointPositions()[0].getAngle(Angle.Unit.RAD) + (Math.PI/2);
		double xContribution = Math.cos(baseAngle);
		double yContribution = Math.sin(baseAngle);

		double angle = -Math.PI;
		double angularStepDistance = (2*Math.PI)/(double) NUMBER_OF_WAYPOINTS;

		for (WaypointNode waypointNode : waypointNodes) {
			angle += angularStepDistance;
			double offsetX = Math.cos(angle) * HORIZONTAL_RADIUS_IN_MM * xContribution;
			double offsetY = Math.cos(angle) * HORIZONTAL_RADIUS_IN_MM * yContribution;
			double offsetZ = Math.sin(-angle) * VERTICAL_RADIUS_IN_MM;

			WaypointNodeConfig newWaypointNodeConfig = createWaypointConfig(centerPose, jointPositions,
					offsetX, offsetY, offsetZ);
			waypointNode.setConfig(newWaypointNodeConfig);
		}
	}

	private WaypointNodeConfig createWaypointConfig(Pose centerPose, JointPositions jointPositions,
	                                                double xOffsetInMM, double yOffsetInMM, double zOffsetInMM) {
		BlendParameters blendParameters = waypointNodeConfigFactory.createSharedBlendParameters();
		WaypointMotionParameters motionParameters = waypointNodeConfigFactory.createSharedMotionParameters();
		Pose pose = createPoseUsingCenterPoseAndOffset(centerPose, xOffsetInMM, yOffsetInMM, zOffsetInMM, Length.Unit.MM);

		return waypointNodeConfigFactory.createFixedPositionConfig(pose, jointPositions, blendParameters, motionParameters);
	}

	private Pose createPoseUsingCenterPoseAndOffset(Pose pose, double xOffset, double yOffset,
	                                                double zOffset, Length.Unit unit) {
		double x = pose.getPosition().getX(unit) + xOffset;
		double y = pose.getPosition().getY(unit) + yOffset;
		double z = pose.getPosition().getZ(unit) + zOffset;
		double rx = pose.getRotation().getRX(Angle.Unit.RAD);
		double ry = pose.getRotation().getRY(Angle.Unit.RAD);
		double rz = pose.getRotation().getRZ(Angle.Unit.RAD);
		return urCapAPI.getValueFactoryProvider().getPoseFactory().createPose(x, y, z, rx, ry, rz, unit, Angle.Unit.RAD);
	}

	private void createNodes() {
		ProgramModel programModel = urCapAPI.getProgramModel();
		try {
			moveNode = programNodeFactory.createMoveNodeNoTemplate();
			TreeNode rootTreeNode = programModel.getRootTreeNode(this);
			moveTreeNode = rootTreeNode.addChild(moveNode);

			waypointNodes.clear();
			for (int i = 1; i <= NUMBER_OF_WAYPOINTS; i++) {
				createAndAddWaypointNode(i);
			}
		} catch (TreeStructureException e) {
			e.printStackTrace();
		}
	}

	private void configureMoveNode() {
		SimpleValueFactory valueFactory = urCapAPI.getValueFactoryProvider().getSimpleValueFactory();

		Speed speed = valueFactory.createSpeed(SHARED_TOOL_SPEED, Speed.Unit.MM_S);
		Acceleration acceleration = valueFactory.createAcceleration(SHARED_TOOL_ACCELERATION, Acceleration.Unit.MM_S2);
		Length length = valueFactory.createLength(SHARED_BLEND_RADIUS_IN_MM, Length.Unit.MM);
		Blend blend = urCapAPI.getValueFactoryProvider().getBlendFactory().createBlend(length);

		MovePMotionParameters motionParameters = moveNode.getConfigFactory().createMovePMotionParameters(
				speed, ErrorHandler.AUTO_CORRECT,
				acceleration, ErrorHandler.AUTO_CORRECT,
				blend, ErrorHandler.AUTO_CORRECT);

		Feature feature = urCapAPI.getFeatures().getBaseFeature();

		moveNode.setConfig(moveNode.getConfigFactory().createMovePConfig(motionParameters, feature));
	}

	private void createAndAddWaypointNode(int waypointNumber) throws TreeStructureException {
		String waypointName = createWaypointName(waypointNumber);
		WaypointNode waypointNode = programNodeFactory.createWaypointNode(waypointName);
		moveTreeNode.addChild(waypointNode);
		waypointNodes.add(waypointNode);
	}

	private static String createWaypointName(int waypointNumber) {
		return "EllipsePos"+waypointNumber;
	}

	@Override
	public void openView() {
		// nothing needs to happen here in this example
	}

	@Override
	public void closeView() {
		// nothing needs to happen here in this example
	}

	@Override
	public String getTitle() {
		return "Ellipse";
	}

	@Override
	public boolean isDefined() {
		return dataModel.get(DEFINED_KEY, false);
	}

	@Override
	public void generateScript(ScriptWriter writer) {
		writer.writeChildren();
	}

	private void lockTreeNodes() {
		ProgramModel programModel = urCapAPI.getProgramModel();
		TreeNode thisTreeNode = programModel.getRootTreeNode(this);
		thisTreeNode.setChildSequenceLocked(true);
		moveTreeNode.setChildSequenceLocked(true);
	}

	private BufferedImage getErrorImage() {
		BufferedImage image;
		try {
			image = ImageIO.read(getClass().getResource("/com/ur/urcap/examples/ellipse/impl/warning-bigger.png"));
		} catch(IOException e) {
			// Should not happen.
			throw new RuntimeException("Unexpected exception while loading icon.", e);
		}
		return image;
	}

	private void clearErrors() {
		if (errorLabel != null) {
			errorLabel.setVisible(false);
		}
		setDefined(true);

	}

	private void setError() {
		if (errorLabel != null) {
			errorLabel.setVisible(true);
			errorLabel.setText("<html>Error: Could not create ellipse movement<br>Try a different center point.</html>");
			errorLabel.setImage(errorIcon);
		}
		setDefined(false);
	}

	private void setDefined(boolean defined) {
		dataModel.set(DEFINED_KEY, defined);
	}
}
