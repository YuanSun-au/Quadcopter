function CodeDefine() { 
this.def = new Array();
this.def["rtDW"] = {file: "crazyflie_c.html",line:25,type:"var"};this.def["rtU"] = {file: "crazyflie_c.html",line:28,type:"var"};this.def["rtY"] = {file: "crazyflie_c.html",line:31,type:"var"};this.def["crazyflie_step"] = {file: "crazyflie_c.html",line:34,type:"fcn"};this.def["crazyflie_initialize"] = {file: "crazyflie_c.html",line:227,type:"fcn"};this.def["crazyflie_terminate"] = {file: "crazyflie_c.html",line:233,type:"fcn"};this.def["DW"] = {file: "crazyflie_h.html",line:35,type:"type"};this.def["ConstP"] = {file: "crazyflie_h.html",line:48,type:"type"};this.def["ExtU"] = {file: "crazyflie_h.html",line:62,type:"type"};this.def["ExtY"] = {file: "crazyflie_h.html",line:76,type:"type"};this.def["rtConstP"] = {file: "crazyflie_data_c.html",line:25,type:"var"};this.def["int8_T"] = {file: "rtwtypes_h.html",line:53,type:"type"};this.def["uint8_T"] = {file: "rtwtypes_h.html",line:54,type:"type"};this.def["int16_T"] = {file: "rtwtypes_h.html",line:55,type:"type"};this.def["uint16_T"] = {file: "rtwtypes_h.html",line:56,type:"type"};this.def["int32_T"] = {file: "rtwtypes_h.html",line:57,type:"type"};this.def["uint32_T"] = {file: "rtwtypes_h.html",line:58,type:"type"};this.def["real32_T"] = {file: "rtwtypes_h.html",line:59,type:"type"};this.def["real64_T"] = {file: "rtwtypes_h.html",line:60,type:"type"};this.def["real_T"] = {file: "rtwtypes_h.html",line:66,type:"type"};this.def["time_T"] = {file: "rtwtypes_h.html",line:67,type:"type"};this.def["boolean_T"] = {file: "rtwtypes_h.html",line:68,type:"type"};this.def["int_T"] = {file: "rtwtypes_h.html",line:69,type:"type"};this.def["uint_T"] = {file: "rtwtypes_h.html",line:70,type:"type"};this.def["ulong_T"] = {file: "rtwtypes_h.html",line:71,type:"type"};this.def["char_T"] = {file: "rtwtypes_h.html",line:72,type:"type"};this.def["uchar_T"] = {file: "rtwtypes_h.html",line:73,type:"type"};this.def["byte_T"] = {file: "rtwtypes_h.html",line:74,type:"type"};this.def["pointer_T"] = {file: "rtwtypes_h.html",line:92,type:"type"};this.def["isInit"] = {file: "crazyflieModel_c.html",line:35,type:"var"};this.def["sensorData"] = {file: "crazyflieModel_c.html",line:36,type:"var"};this.def["setpoint"] = {file: "crazyflieModel_c.html",line:37,type:"var"};this.def["state"] = {file: "crazyflieModel_c.html",line:38,type:"var"};this.def["l6"] = {file: "crazyflieModel_c.html",line:39,type:"var"};this.def["m4"] = {file: "crazyflieModel_c.html",line:40,type:"var"};this.def["crazyflieModelInit"] = {file: "crazyflieModel_c.html",line:41,type:"fcn"};this.def["crazyflieModelTest"] = {file: "crazyflieModel_c.html",line:62,type:"fcn"};this.def["crazyflieModelTask"] = {file: "crazyflieModel_c.html",line:70,type:"fcn"};}
CodeDefine.instance = new CodeDefine();
var testHarnessInfo = {OwnerFileName: "", HarnessOwner: "", HarnessName: "", IsTestHarness: "0"};
var relPathToBuildDir = "../ert_main.c";
var fileSep = "\\";
var isPC = true;
function Html2SrcLink() {
	this.html2SrcPath = new Array;
	this.html2Root = new Array;
	this.html2SrcPath["crazyflie_c.html"] = "../crazyflie.c";
	this.html2Root["crazyflie_c.html"] = "crazyflie_c.html";
	this.html2SrcPath["crazyflie_h.html"] = "../crazyflie.h";
	this.html2Root["crazyflie_h.html"] = "crazyflie_h.html";
	this.html2SrcPath["crazyflie_data_c.html"] = "../crazyflie_data.c";
	this.html2Root["crazyflie_data_c.html"] = "crazyflie_data_c.html";
	this.html2SrcPath["rtwtypes_h.html"] = "../rtwtypes.h";
	this.html2Root["rtwtypes_h.html"] = "rtwtypes_h.html";
	this.html2SrcPath["rtmodel_h.html"] = "../rtmodel.h";
	this.html2Root["rtmodel_h.html"] = "rtmodel_h.html";
	this.html2SrcPath["crazyflieModel_c.html"] = "../crazyflieModel.c";
	this.html2Root["crazyflieModel_c.html"] = "crazyflieModel_c.html";
	this.html2SrcPath["crazyflieModel_h.html"] = "../crazyflieModel.h";
	this.html2Root["crazyflieModel_h.html"] = "crazyflieModel_h.html";
	this.getLink2Src = function (htmlFileName) {
		 if (this.html2SrcPath[htmlFileName])
			 return this.html2SrcPath[htmlFileName];
		 else
			 return null;
	}
	this.getLinkFromRoot = function (htmlFileName) {
		 if (this.html2Root[htmlFileName])
			 return this.html2Root[htmlFileName];
		 else
			 return null;
	}
}
Html2SrcLink.instance = new Html2SrcLink();
var fileList = [
"crazyflie_c.html","crazyflie_h.html","crazyflie_data_c.html","rtwtypes_h.html","rtmodel_h.html","crazyflieModel_c.html","crazyflieModel_h.html"];
