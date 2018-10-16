package com.ur.urcap.examples.ellipse.impl;

import com.ur.urcap.api.contribution.ProgramNodeContribution;
import com.ur.urcap.api.contribution.ProgramNodeService;
import com.ur.urcap.api.domain.URCapAPI;
import com.ur.urcap.api.domain.data.DataModel;

import java.io.InputStream;

public class EllipseProgramNodeService implements ProgramNodeService {

	@Override
	public String getId() {
		return "Ellipse";
	}

	@Override
	public String getTitle() {
		return "Ellipse";
	}

	@Override
	public InputStream getHTML() {
		return this.getClass().getResourceAsStream("/com/ur/urcap/examples/ellipse/impl/EllipseProgramNode.html");
	}

	@Override
	public boolean isDeprecated() {
		return false;
	}

	@Override
	public boolean isChildrenAllowed() {
		return true;
	}

	@Override
	public ProgramNodeContribution createNode(URCapAPI urCapAPI, DataModel dataModel) {
		return new EllipseProgramNodeContribution(urCapAPI, dataModel);
	}
}
