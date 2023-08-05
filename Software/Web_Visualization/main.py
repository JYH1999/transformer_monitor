from distutils.log import debug
import socket
from threading import Thread
#from pyecharts.globals import CurrentConfig
#CurrentConfig.ONLINE_HOST = "http://8.136.87.151:8000/assets/"
from pyecharts.charts import Map,Geo
import pyecharts.options as opts
from pyecharts.charts import Gauge
from pyecharts.charts import Line
from pyecharts.charts import Bar
from pyecharts.commons.utils import JsCode
from pyecharts.globals import ThemeType
from pyecharts.charts import Pie
from pyecharts.faker import Faker
from pyecharts.charts import Liquid
from pyecharts.charts import Tab,Page,Grid
import re
import time
from flask import Flask
from flask import render_template

#调试模式
debug_mode=True
#参数
param_a={
    "AIWAVE":[0],
    "AVWAVE":[0],
    "AIRMS":[0],
    "AVRMS":[0],
    "AWATT":[0],
    "AWAHR":[0],
    "AVAR":[0],
    "AVAHR":[0]}
param_b={
    "BIWAVE":[0],
    "BVWAVE":[0],
    "BIRMS":[0],
    "BVRMS":[0],
    "BWATT":[0],
    "BWAHR":[0],
    "BVAR":[0],
    "BVAHR":[0]}
param_c={
    "CIWAVE":[0],
    "CVWAVE":[0],
    "CIRMS":[0],
    "CVRMS":[0],
    "CWATT":[0],
    "CWAHR":[0],
    "CVAR":[0],
    "CVAHR":[0]}
param_n={"NIWAVE":[0],"NIRMS":[0]}
param_env={"Temp":[0],"Humid":[0]}

#UDP解包函数，输入udp_socket元组，返回udp_message类型列表
def udp_unpack(udp_package,dbg_mode=False,codec='utf-8'):
    if dbg_mode==True:
        print("Funcion:udp_unpack")
    try:
        return_message=[udp_package[0].decode(codec),str(udp_package[1])]
        if dbg_mode==True:
            print("udp_message:",end="")
            print(return_message)
    except:
        if dbg_mode==True:
            print("Exception happend! Failed to unpack UDP package!")
        return
    return return_message

#数据解析函数，输入udp_message类型列表，返回raw_message类型数据列表
def message_parser(udp_message=[],dbg_mode=False):
    raw_message=[]
    if dbg_mode==True:
        print("Function:message_parser")
    pattern=r'(.*)=(\-?\d+\.?\d+).*'
    parser_obj=re.match(pattern,udp_message[0],re.M|re.I)
    message_value=eval(parser_obj.group(2))
    raw_message.append(parser_obj.group(1))
    raw_message.append(message_value)
    if dbg_mode==True:
        print("message_value:"+str(message_value))
        print("raw_message:",end="")
        print(raw_message)
    return raw_message

#参数转换函数，输入raw_message类型数据列表，将数据直接写入param_x字典中
def param_converter(raw_message=[],dbg_mode=False):
    if dbg_mode==True:
        print("Function:param_converter")
    if raw_message[0][0]=="A":
        if len(param_a[raw_message[0]])>15:
            param_a[raw_message[0]].pop(0)
        param_a[raw_message[0]].append(raw_message[1])
        if dbg_mode==True:
            print("Param A detected, convert data to param_a.")
            print("param_a:",end="")
            print(param_a)
    if raw_message[0][0]=="B":
        if len(param_b[raw_message[0]])>15:
            param_b[raw_message[0]].pop(0)
        param_b[raw_message[0]].append(raw_message[1])
        if dbg_mode==True:
            print("Param B detected, convert data to param_b.")
            print("param_b:",end="")
            print(param_b)
    if raw_message[0][0]=="C":
        if len(param_c[raw_message[0]])>15:
            param_c[raw_message[0]].pop(0)
        param_c[raw_message[0]].append(raw_message[1])
        if dbg_mode==True:
            print("Param C detected, convert data to param_c.")
            print("param_c:",end="")
            print(param_c)
    if raw_message[0][0]=="N":
        if len(param_n[raw_message[0]])>15:
            param_n[raw_message[0]].pop(0)
        param_n[raw_message[0]].append(raw_message[1])
        if dbg_mode==True:
            print("Param N detected, convert data to param_n.")
            print("param_n:",end="")
            print(param_n)
    if raw_message[0][0]=="T" or raw_message[0][0]=="H":
        if len(param_env[raw_message[0]])>15:
            param_env[raw_message[0]].pop(0)
        param_env[raw_message[0]].append(raw_message[1])
        if dbg_mode==True:
            print("Param ENV detected, convert data to param_env.")
            print("param_env:",end="")
            print(param_env)

#UDP线程函数    
def udp_threads():
    if debug_mode==True:
        print("Function:udp_threads")
        print("Start UDP thread!")
    try:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        local_address = ("0.0.0.0", 4950)
        udp_socket.bind(local_address)
        if debug_mode==True:
            print("Establish UDP socket successfully!")
            print("UDP socket:",end="")
            print(local_address)
    except:
        print("ERROR:Failed to establish UDP socket!")
        return
    if debug_mode==True:
        print("Start receiving data from UDP socket...")
    while True:
        try:
            # 接收数据
            receive_data = udp_socket.recvfrom(1024)
            # receive_data 这个变量存储的是一个元组，(接收到的数据，(发送方的ip, port))
            udp_message=udp_unpack(receive_data,dbg_mode=debug_mode)
            raw_message=message_parser(udp_message,dbg_mode=debug_mode)
            param_converter(raw_message,dbg_mode=debug_mode)
        except:
            print("Exception happened in UDP receive process!")
            continue

#实时温度仪表盘
def get_temp_gauge():
    temp_gauge=(
        Gauge()
        .add(series_name="实时温度", data_pair=[["实时温度", param_env["Temp"][-1]]],detail_label_opts=opts.LabelOpts(formatter="{value}℃"))
        .set_global_opts(
            title_opts=opts.TitleOpts(title="统计",pos_right="center",pos_top="40%",title_link='/temp_page/',title_target='self'),
            legend_opts=opts.LegendOpts(is_show=False),
            tooltip_opts=opts.TooltipOpts(is_show=False, formatter="{a} <br/>{b} : {c}℃"),
        )
    )
    temp_gauge.chart_id='tempgauge'
    return temp_gauge

#实时湿度仪表盘
def get_humid_gauge():
    humid_gauge=(
        Gauge()
        .add(series_name="实时湿度", data_pair=[["实时湿度", param_env["Humid"][-1]]],detail_label_opts=opts.LabelOpts(formatter="{value}%"))
        .set_global_opts(
            title_opts=opts.TitleOpts(title="统计",pos_right="center",pos_top="40%",title_link='/humid_page/',title_target='self'),
            legend_opts=opts.LegendOpts(is_show=False),
            tooltip_opts=opts.TooltipOpts(is_show=False, formatter="{a} <br/>{b} : {c}%"),
        )
    )
    humid_gauge.chart_id='humidgauge'
    return humid_gauge

#实时AVRMS仪表盘
def get_avrms_gauge():
    avrms_gauge=(
        Gauge()
        .add(series_name="AVRMS", min_=0,max_=600,data_pair=[["AVRMS", param_a["AVRMS"][-1]]],detail_label_opts=opts.LabelOpts(formatter="{value}V"))
        .set_global_opts(
            title_opts=opts.TitleOpts(title="统计",pos_right="center",pos_top="40%",title_link='/avrms_page/',title_target='self'),
            legend_opts=opts.LegendOpts(is_show=False),
            tooltip_opts=opts.TooltipOpts(is_show=False, formatter="{a} <br/>{b} : {c}%"),
        )
    )
    avrms_gauge.chart_id='avrmsgauge'
    return avrms_gauge

#实时BVRMS仪表盘
def get_bvrms_gauge():
    bvrms_gauge=(
        Gauge()
        .add(series_name="BVRMS", min_=0,max_=600,data_pair=[["BVRMS", param_b["BVRMS"][-1]]],detail_label_opts=opts.LabelOpts(formatter="{value}V"))
        .set_global_opts(
            title_opts=opts.TitleOpts(title="统计",pos_right="center",pos_top="40%",title_link='/bvrms_page/',title_target='self'),
            legend_opts=opts.LegendOpts(is_show=False),
            tooltip_opts=opts.TooltipOpts(is_show=False, formatter="{a} <br/>{b} : {c}%"),
        )
    )
    bvrms_gauge.chart_id='bvrmsgauge'
    return bvrms_gauge

#实时CVRMS仪表盘
def get_cvrms_gauge():
    cvrms_gauge=(
        Gauge()
        .add(series_name="CVRMS", min_=0,max_=600,data_pair=[["CVRMS", param_c["CVRMS"][-1]]],detail_label_opts=opts.LabelOpts(formatter="{value}V"))
        .set_global_opts(
            title_opts=opts.TitleOpts(title="统计",pos_right="center",pos_top="40%",title_link='/cvrms_page/',title_target='self'),
            legend_opts=opts.LegendOpts(is_show=False),
            tooltip_opts=opts.TooltipOpts(is_show=False, formatter="{a} <br/>{b} : {c}%"),
        )
    )
    cvrms_gauge.chart_id='cvrmsgauge'
    return cvrms_gauge

#实时AIRMS仪表盘
def get_airms_gauge():
    airms_gauge=(
        Gauge()
        .add(series_name="AIRMS", min_=0,max_=75,data_pair=[["AIRMS", param_a["AIRMS"][-1]]],detail_label_opts=opts.LabelOpts(formatter="{value}A"))
        .set_global_opts(
            title_opts=opts.TitleOpts(title="统计",pos_right="center",pos_top="40%",title_link='/airms_page/',title_target='self'),
            legend_opts=opts.LegendOpts(is_show=False),
            tooltip_opts=opts.TooltipOpts(is_show=False, formatter="{a} <br/>{b} : {c}%"),
        )
    )
    airms_gauge.chart_id='airmsgauge'
    return airms_gauge

#实时BIRMS仪表盘
def get_birms_gauge():
    birms_gauge=(
        Gauge()
        .add(series_name="BIRMS",min_=0,max_=75, data_pair=[["BIRMS", param_b["BIRMS"][-1]]],detail_label_opts=opts.LabelOpts(formatter="{value}A"))
        .set_global_opts(
            title_opts=opts.TitleOpts(title="统计",pos_right="center",pos_top="40%",title_link='/birms_page/',title_target='self'),
            legend_opts=opts.LegendOpts(is_show=False),
            tooltip_opts=opts.TooltipOpts(is_show=False, formatter="{a} <br/>{b} : {c}%"),
        )
    )
    birms_gauge.chart_id='birmsgauge'
    return birms_gauge

#实时CIRMS仪表盘
def get_cirms_gauge():
    cirms_gauge=(
        Gauge()
        .add(series_name="CIRMS",min_=0,max_=75, data_pair=[["CIRMS", param_c["CIRMS"][-1]]],detail_label_opts=opts.LabelOpts(formatter="{value}A"))
        .set_global_opts(
            title_opts=opts.TitleOpts(title="统计",pos_right="center",pos_top="40%",title_link='/cirms_page/',title_target='self'),
            legend_opts=opts.LegendOpts(is_show=False),
            tooltip_opts=opts.TooltipOpts(is_show=False, formatter="{a} <br/>{b} : {c}%"),
        )
    )
    cirms_gauge.chart_id='cirmsgauge'
    return cirms_gauge

#实时NIRMS仪表盘
def get_nirms_gauge():
    nirms_gauge=(
        Gauge()
        .add(series_name="NIRMS",min_=0,max_=75, data_pair=[["NIRMS", param_n["NIRMS"][-1]]],detail_label_opts=opts.LabelOpts(formatter="{value}A"))
        .set_global_opts(
            title_opts=opts.TitleOpts(title="统计",pos_right="center",pos_top="40%",title_link='/nirms_page/',title_target='self'),
            legend_opts=opts.LegendOpts(is_show=False),
            tooltip_opts=opts.TooltipOpts(is_show=False, formatter="{a} <br/>{b} : {c}%"),
        )
    )
    nirms_gauge.chart_id='nirmsgauge'
    return nirms_gauge

#实时AWATT仪表盘
def get_awatt_gauge():
    awatt_gauge=(
        Gauge()
        .add(series_name="AWATT",min_=0,max_=42000, data_pair=[["AWATT", param_a["AWATT"][-1]]],detail_label_opts=opts.LabelOpts(formatter="{value}W"))
        .set_global_opts(
            title_opts=opts.TitleOpts(title="统计",pos_right="center",pos_top="40%",title_link='/awatt_page/',title_target='self'),
            legend_opts=opts.LegendOpts(is_show=False),
            tooltip_opts=opts.TooltipOpts(is_show=False, formatter="{a} <br/>{b} : {c}%"),
        )
    )
    awatt_gauge.chart_id='awattgauge'
    return awatt_gauge

#实时BWATT仪表盘
def get_bwatt_gauge():
    bwatt_gauge=(
        Gauge()
        .add(series_name="BWATT",min_=0,max_=42000, data_pair=[["BWATT", param_b["BWATT"][-1]]],detail_label_opts=opts.LabelOpts(formatter="{value}W"))
        .set_global_opts(
            title_opts=opts.TitleOpts(title="统计",pos_right="center",pos_top="40%",title_link='/bwatt_page/',title_target='self'),
            legend_opts=opts.LegendOpts(is_show=False),
            tooltip_opts=opts.TooltipOpts(is_show=False, formatter="{a} <br/>{b} : {c}%"),
        )
    )
    bwatt_gauge.chart_id='bwattgauge'
    return bwatt_gauge

#实时CWATT仪表盘
def get_cwatt_gauge():
    cwatt_gauge=(
        Gauge()
        .add(series_name="CWATT",min_=0,max_=42000, data_pair=[["CWATT", param_c["CWATT"][-1]]],detail_label_opts=opts.LabelOpts(formatter="{value}W"))
        .set_global_opts(
            title_opts=opts.TitleOpts(title="统计",pos_right="center",pos_top="40%",title_link='/cwatt_page/',title_target='self'),
            legend_opts=opts.LegendOpts(is_show=False),
            tooltip_opts=opts.TooltipOpts(is_show=False, formatter="{a} <br/>{b} : {c}%"),
        )
    )
    cwatt_gauge.chart_id='cwattgauge'
    return cwatt_gauge

#实时AVAR仪表盘
def get_avar_gauge():
    avar_gauge=(
        Gauge()
        .add(series_name="AVAR",min_=0,max_=1000, data_pair=[["AVAR", param_a["AVAR"][-1]]],detail_label_opts=opts.LabelOpts(formatter="{value}VAR"))
        .set_global_opts(
            title_opts=opts.TitleOpts(title="统计",pos_right="center",pos_top="40%",title_link='/avar_page/',title_target='self'),
            legend_opts=opts.LegendOpts(is_show=False),
            tooltip_opts=opts.TooltipOpts(is_show=False, formatter="{a} <br/>{b} : {c}%"),
        )
    )
    avar_gauge.chart_id='avargauge'
    return avar_gauge

#实时BVAR仪表盘
def get_bvar_gauge():
    bvar_gauge=(
        Gauge()
        .add(series_name="BVAR",min_=0,max_=1000, data_pair=[["BVAR", param_b["BVAR"][-1]]],detail_label_opts=opts.LabelOpts(formatter="{value}VAR"))
        .set_global_opts(
            title_opts=opts.TitleOpts(title="统计",pos_right="center",pos_top="40%",title_link='/bvar_page/',title_target='self'),
            legend_opts=opts.LegendOpts(is_show=False),
            tooltip_opts=opts.TooltipOpts(is_show=False, formatter="{a} <br/>{b} : {c}%"),
        )
    )
    bvar_gauge.chart_id='bvargauge'
    return bvar_gauge

#实时CVAR仪表盘
def get_cvar_gauge():
    cvar_gauge=(
        Gauge()
        .add(series_name="CVAR",min_=0,max_=1000, data_pair=[["CVAR", param_c["CVAR"][-1]]],detail_label_opts=opts.LabelOpts(formatter="{value}VAR"))
        .set_global_opts(
            title_opts=opts.TitleOpts(title="统计",pos_right="center",pos_top="40%",title_link='/cvar_page/',title_target='self'),
            legend_opts=opts.LegendOpts(is_show=False),
            tooltip_opts=opts.TooltipOpts(is_show=False, formatter="{a} <br/>{b} : {c}%"),
        )
    )
    cvar_gauge.chart_id='cvargauge'
    return cvar_gauge

#温度历史折线图
def get_temp_line():
    temp_line=(
        Line()
        .add_xaxis(xaxis_data=[str(m) for m in range(1,len(param_env["Temp"])+1)])
        .add_yaxis(
            series_name="温度(℃)",
            y_axis=param_env["Temp"],
            markpoint_opts=opts.MarkPointOpts(
                data=[
                    opts.MarkPointItem(type_="max", name="最大值"),
                    opts.MarkPointItem(type_="min", name="最小值"),
                ]
            ),
            markline_opts=opts.MarkLineOpts(
                data=[opts.MarkLineItem(type_="average", name="平均值")]
            ),
        )
        .set_global_opts(
            # title_opts=opts.TitleOpts(title="温度检测", subtitle="历史数据"),
            tooltip_opts=opts.TooltipOpts(trigger="axis"),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            xaxis_opts=opts.AxisOpts(name="记录",type_="category", boundary_gap=False),
            yaxis_opts=opts.AxisOpts(name="温度(℃)"),
        )   
    )
    temp_line.chart_id="templine"
    return temp_line

#湿度历史折线图
def get_humid_line():
    humid_line=(
        Line(init_opts=opts.InitOpts())
        .add_xaxis(xaxis_data=[str(m) for m in range(1,len(param_env["Humid"])+1)])
        .add_yaxis(
            series_name="湿度(%)",
            y_axis=param_env["Humid"],
            markpoint_opts=opts.MarkPointOpts(
                data=[
                    opts.MarkPointItem(type_="max", name="最大值"),
                    opts.MarkPointItem(type_="min", name="最小值"),
                ]
            ),
            markline_opts=opts.MarkLineOpts(
                data=[opts.MarkLineItem(type_="average", name="平均值")]
            ),
        )
        .set_global_opts(
            # title_opts=opts.TitleOpts(title="湿度检测", subtitle="历史数据"),
            tooltip_opts=opts.TooltipOpts(trigger="axis"),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            xaxis_opts=opts.AxisOpts(name="记录",type_="category", boundary_gap=False),
            yaxis_opts=opts.AxisOpts(name="湿度(%)"),
        )   
    )
    humid_line.chart_id="humidline"
    return humid_line

#AVRMS折线图
def get_avrms_line():
    avrms_line=(
        Line(init_opts=opts.InitOpts())
        .add_xaxis(xaxis_data=[str(m) for m in range(1,len(param_a["AVRMS"])+1)])
        .add_yaxis(
            series_name="AVRMS(V)",
            y_axis=param_a["AVRMS"],
            markpoint_opts=opts.MarkPointOpts(
                data=[
                    opts.MarkPointItem(type_="max", name="最大值"),
                    opts.MarkPointItem(type_="min", name="最小值"),
                ]
            ),
            markline_opts=opts.MarkLineOpts(
                data=[opts.MarkLineItem(type_="average", name="平均值")]
            ),
        )
        .set_global_opts(
            tooltip_opts=opts.TooltipOpts(trigger="axis"),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            xaxis_opts=opts.AxisOpts(name="记录",type_="category", boundary_gap=False),
            yaxis_opts=opts.AxisOpts(name="AVRMS(V)"),
        )   
    )
    avrms_line.chart_id="avrmsline"
    return avrms_line

#AIRMS折线图
def get_airms_line():
    airms_line=(
        Line(init_opts=opts.InitOpts())
        .add_xaxis(xaxis_data=[str(m) for m in range(1,len(param_a["AIRMS"])+1)])
        .add_yaxis(
            series_name="AIRMS(A)",
            y_axis=param_a["AIRMS"],
            markpoint_opts=opts.MarkPointOpts(
                data=[
                    opts.MarkPointItem(type_="max", name="最大值"),
                    opts.MarkPointItem(type_="min", name="最小值"),
                ]
            ),
            markline_opts=opts.MarkLineOpts(
                data=[opts.MarkLineItem(type_="average", name="平均值")]
            ),
        )
        .set_global_opts(
            tooltip_opts=opts.TooltipOpts(trigger="axis"),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            xaxis_opts=opts.AxisOpts(name="记录",type_="category", boundary_gap=False),
            yaxis_opts=opts.AxisOpts(name="AIRMS(A)"),
        )   
    )
    airms_line.chart_id="airmsline"
    return airms_line

#AWATT折线图
def get_awatt_line():
    awatt_line=(
        Line(init_opts=opts.InitOpts())
        .add_xaxis(xaxis_data=[str(m) for m in range(1,len(param_a["AWATT"])+1)])
        .add_yaxis(
            series_name="AWATT(W)",
            y_axis=param_a["AWATT"],
            markpoint_opts=opts.MarkPointOpts(
                data=[
                    opts.MarkPointItem(type_="max", name="最大值"),
                    opts.MarkPointItem(type_="min", name="最小值"),
                ]
            ),
            markline_opts=opts.MarkLineOpts(
                data=[opts.MarkLineItem(type_="average", name="平均值")]
            ),
        )
        .set_global_opts(
            tooltip_opts=opts.TooltipOpts(trigger="axis"),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            xaxis_opts=opts.AxisOpts(name="记录",type_="category", boundary_gap=False),
            yaxis_opts=opts.AxisOpts(name="AWATT(W)"),
        )   
    )
    awatt_line.chart_id="awattline"
    return awatt_line

#AVAR折线图
def get_avar_line():
    avar_line=(
        Line(init_opts=opts.InitOpts())
        .add_xaxis(xaxis_data=[str(m) for m in range(1,len(param_a["AVAR"])+1)])
        .add_yaxis(
            series_name="AVAR(VAR)",
            y_axis=param_a["AVAR"],
            markpoint_opts=opts.MarkPointOpts(
                data=[
                    opts.MarkPointItem(type_="max", name="最大值"),
                    opts.MarkPointItem(type_="min", name="最小值"),
                ]
            ),
            markline_opts=opts.MarkLineOpts(
                data=[opts.MarkLineItem(type_="average", name="平均值")]
            ),
        )
        .set_global_opts(
            tooltip_opts=opts.TooltipOpts(trigger="axis"),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            xaxis_opts=opts.AxisOpts(name="记录",type_="category", boundary_gap=False),
            yaxis_opts=opts.AxisOpts(name="AVAR(VAR)"),
        )   
    )
    avar_line.chart_id="avarline"
    return avar_line

#BVRMS折线图
def get_bvrms_line():
    bvrms_line=(
        Line(init_opts=opts.InitOpts())
        .add_xaxis(xaxis_data=[str(m) for m in range(1,len(param_b["BVRMS"])+1)])
        .add_yaxis(
            series_name="BVRMS(V)",
            y_axis=param_b["BVRMS"],
            markpoint_opts=opts.MarkPointOpts(
                data=[
                    opts.MarkPointItem(type_="max", name="最大值"),
                    opts.MarkPointItem(type_="min", name="最小值"),
                ]
            ),
            markline_opts=opts.MarkLineOpts(
                data=[opts.MarkLineItem(type_="average", name="平均值")]
            ),
        )
        .set_global_opts(
            tooltip_opts=opts.TooltipOpts(trigger="axis"),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            xaxis_opts=opts.AxisOpts(name="记录",type_="category", boundary_gap=False),
            yaxis_opts=opts.AxisOpts(name="BVRMS(V)"),
        )   
    )
    bvrms_line.chart_id="bvrmsline"
    return bvrms_line

#BIRMS折线图
def get_birms_line():
    birms_line=(
        Line(init_opts=opts.InitOpts())
        .add_xaxis(xaxis_data=[str(m) for m in range(1,len(param_b["BIRMS"])+1)])
        .add_yaxis(
            series_name="BIRMS(A)",
            y_axis=param_b["BIRMS"],
            markpoint_opts=opts.MarkPointOpts(
                data=[
                    opts.MarkPointItem(type_="max", name="最大值"),
                    opts.MarkPointItem(type_="min", name="最小值"),
                ]
            ),
            markline_opts=opts.MarkLineOpts(
                data=[opts.MarkLineItem(type_="average", name="平均值")]
            ),
        )
        .set_global_opts(
            tooltip_opts=opts.TooltipOpts(trigger="axis"),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            xaxis_opts=opts.AxisOpts(name="记录",type_="category", boundary_gap=False),
            yaxis_opts=opts.AxisOpts(name="BIRMS(A)"),
        )   
    )
    birms_line.chart_id="birmsline"
    return birms_line

#BWATT折线图
def get_bwatt_line():
    bwatt_line=(
        Line(init_opts=opts.InitOpts())
        .add_xaxis(xaxis_data=[str(m) for m in range(1,len(param_b["BWATT"])+1)])
        .add_yaxis(
            series_name="BWATT(W)",
            y_axis=param_b["BWATT"],
            markpoint_opts=opts.MarkPointOpts(
                data=[
                    opts.MarkPointItem(type_="max", name="最大值"),
                    opts.MarkPointItem(type_="min", name="最小值"),
                ]
            ),
            markline_opts=opts.MarkLineOpts(
                data=[opts.MarkLineItem(type_="average", name="平均值")]
            ),
        )
        .set_global_opts(
            tooltip_opts=opts.TooltipOpts(trigger="axis"),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            xaxis_opts=opts.AxisOpts(name="记录",type_="category", boundary_gap=False),
            yaxis_opts=opts.AxisOpts(name="BWATT(W)"),
        )   
    )
    bwatt_line.chart_id="bwattline"
    return bwatt_line

#BVAR折线图
def get_bvar_line():
    bvar_line=(
        Line(init_opts=opts.InitOpts())
        .add_xaxis(xaxis_data=[str(m) for m in range(1,len(param_b["BVAR"])+1)])
        .add_yaxis(
            series_name="BVAR(VAR)",
            y_axis=param_b["BVAR"],
            markpoint_opts=opts.MarkPointOpts(
                data=[
                    opts.MarkPointItem(type_="max", name="最大值"),
                    opts.MarkPointItem(type_="min", name="最小值"),
                ]
            ),
            markline_opts=opts.MarkLineOpts(
                data=[opts.MarkLineItem(type_="average", name="平均值")]
            ),
        )
        .set_global_opts(
            tooltip_opts=opts.TooltipOpts(trigger="axis"),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            xaxis_opts=opts.AxisOpts(name="记录",type_="category", boundary_gap=False),
            yaxis_opts=opts.AxisOpts(name="BVAR(VAR)"),
        )   
    )
    bvar_line.chart_id="bvarline"
    return bvar_line

#CVRMS折线图
def get_cvrms_line():
    cvrms_line=(
        Line(init_opts=opts.InitOpts())
        .add_xaxis(xaxis_data=[str(m) for m in range(1,len(param_c["CVRMS"])+1)])
        .add_yaxis(
            series_name="CVRMS(V)",
            y_axis=param_c["CVRMS"],
            markpoint_opts=opts.MarkPointOpts(
                data=[
                    opts.MarkPointItem(type_="max", name="最大值"),
                    opts.MarkPointItem(type_="min", name="最小值"),
                ]
            ),
            markline_opts=opts.MarkLineOpts(
                data=[opts.MarkLineItem(type_="average", name="平均值")]
            ),
        )
        .set_global_opts(
            tooltip_opts=opts.TooltipOpts(trigger="axis"),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            xaxis_opts=opts.AxisOpts(name="记录",type_="category", boundary_gap=False),
            yaxis_opts=opts.AxisOpts(name="CVRMS(V)"),
        )   
    )
    cvrms_line.chart_id="cvrmsline"
    return cvrms_line

#CIRMS折线图
def get_cirms_line():
    cirms_line=(
        Line(init_opts=opts.InitOpts())
        .add_xaxis(xaxis_data=[str(m) for m in range(1,len(param_c["CIRMS"])+1)])
        .add_yaxis(
            series_name="CIRMS(A)",
            y_axis=param_c["CIRMS"],
            markpoint_opts=opts.MarkPointOpts(
                data=[
                    opts.MarkPointItem(type_="max", name="最大值"),
                    opts.MarkPointItem(type_="min", name="最小值"),
                ]
            ),
            markline_opts=opts.MarkLineOpts(
                data=[opts.MarkLineItem(type_="average", name="平均值")]
            ),
        )
        .set_global_opts(
            tooltip_opts=opts.TooltipOpts(trigger="axis"),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            xaxis_opts=opts.AxisOpts(name="记录",type_="category", boundary_gap=False),
            yaxis_opts=opts.AxisOpts(name="CIRMS(A)"),
        )   
    )
    cirms_line.chart_id="cirmsline"
    return cirms_line

#CWATT折线图
def get_cwatt_line():
    cwatt_line=(
        Line(init_opts=opts.InitOpts())
        .add_xaxis(xaxis_data=[str(m) for m in range(1,len(param_c["CWATT"])+1)])
        .add_yaxis(
            series_name="CWATT(W)",
            y_axis=param_c["CWATT"],
            markpoint_opts=opts.MarkPointOpts(
                data=[
                    opts.MarkPointItem(type_="max", name="最大值"),
                    opts.MarkPointItem(type_="min", name="最小值"),
                ]
            ),
            markline_opts=opts.MarkLineOpts(
                data=[opts.MarkLineItem(type_="average", name="平均值")]
            ),
        )
        .set_global_opts(
            tooltip_opts=opts.TooltipOpts(trigger="axis"),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            xaxis_opts=opts.AxisOpts(name="记录",type_="category", boundary_gap=False),
            yaxis_opts=opts.AxisOpts(name="CWATT(W)"),
        )   
    )
    cwatt_line.chart_id="cwattline"
    return cwatt_line

#CVAR折线图
def get_cvar_line():
    cvar_line=(
        Line(init_opts=opts.InitOpts())
        .add_xaxis(xaxis_data=[str(m) for m in range(1,len(param_c["CVAR"])+1)])
        .add_yaxis(
            series_name="CVAR(VAR)",
            y_axis=param_c["CVAR"],
            markpoint_opts=opts.MarkPointOpts(
                data=[
                    opts.MarkPointItem(type_="max", name="最大值"),
                    opts.MarkPointItem(type_="min", name="最小值"),
                ]
            ),
            markline_opts=opts.MarkLineOpts(
                data=[opts.MarkLineItem(type_="average", name="平均值")]
            ),
        )
        .set_global_opts(
            tooltip_opts=opts.TooltipOpts(trigger="axis"),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            xaxis_opts=opts.AxisOpts(name="记录",type_="category", boundary_gap=False),
            yaxis_opts=opts.AxisOpts(name="CVAR(VAR)"),
        )   
    )
    cvar_line.chart_id="cvarline"
    return cvar_line

#NIRMS折线图
def get_nirms_line():
    nirms_line=(
        Line(init_opts=opts.InitOpts())
        .add_xaxis(xaxis_data=[str(m) for m in range(1,len(param_n["NIRMS"])+1)])
        .add_yaxis(
            series_name="NIRMS(A)",
            y_axis=param_n["NIRMS"],
            markpoint_opts=opts.MarkPointOpts(
                data=[
                    opts.MarkPointItem(type_="max", name="最大值"),
                    opts.MarkPointItem(type_="min", name="最小值"),
                ]
            ),
            markline_opts=opts.MarkLineOpts(
                data=[opts.MarkLineItem(type_="average", name="平均值")]
            ),
        )
        .set_global_opts(
            tooltip_opts=opts.TooltipOpts(trigger="axis"),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            xaxis_opts=opts.AxisOpts(name="记录",type_="category", boundary_gap=False),
            yaxis_opts=opts.AxisOpts(name="NIRMS(A)"),
        )   
    )
    nirms_line.chart_id="nirmsline"
    return nirms_line

#生成首页
def generate_main_page(dbg_mode=False):
    if dbg_mode==True:
        print("Function:generate_main_page")
    main_page=Page(layout=Page.DraggablePageLayout)
    main_page.add(
        get_temp_gauge(),
        get_humid_gauge(),
        get_avrms_gauge(),
        get_bvrms_gauge(),
        get_cvrms_gauge(),
        get_airms_gauge(),
        get_birms_gauge(),
        get_cirms_gauge(),
        get_nirms_gauge(),
        get_awatt_gauge(),
        get_bwatt_gauge(),
        get_cwatt_gauge(),
        get_avar_gauge(),
        get_bvar_gauge(),
        get_cvar_gauge()
    )
    if dbg_mode==True:
        print("Render resizable page...")
    main_page.render('./temp_pages/main_page_temp.html')
    if dbg_mode==True:
        print("Render main page...")
    Page.save_resize_html('./temp_pages/main_page_temp.html',cfg_file='./config_json/main_page.json',dest='./templates/main_page.html')

#生成温度页
def generate_temp_page(dbg_mode=False):
    if dbg_mode==True:
        print("Function:generate_temp_page")
    temp_page=Page(layout=Page.DraggablePageLayout)
    temp_page.add(
        get_temp_gauge(),
        get_temp_line()
    )
    if dbg_mode==True:
        print("Render resizable page...")
    temp_page.render('./temp_pages/temp_page_temp.html')
    if dbg_mode==True:
        print("Render temp page...")
    Page.save_resize_html('./temp_pages/temp_page_temp.html',cfg_file='./config_json/temp_page.json',dest='./templates/temp_page.html')

#生成湿度页
def generate_humid_page(dbg_mode=False):
    if dbg_mode==True:
        print("Function:generate_humid_page")
    humid_page=Page(layout=Page.DraggablePageLayout)
    humid_page.add(
        get_humid_gauge(),
        get_humid_line()
    )
    if dbg_mode==True:
        print("Render resizable page...")
    humid_page.render('./temp_pages/humid_page_temp.html')
    if dbg_mode==True:
        print("Render humid page...")
    Page.save_resize_html('./temp_pages/humid_page_temp.html',cfg_file='./config_json/humid_page.json',dest='./templates/humid_page.html')

#生成AVRMS页
def generate_avrms_page(dbg_mode=False):
    if dbg_mode==True:
        print("Function:generate_avrms_page")
    avrms_page=Page(layout=Page.DraggablePageLayout)
    avrms_page.add(
        get_avrms_gauge(),
        get_avrms_line()
    )
    if dbg_mode==True:
        print("Render resizable page...")
    avrms_page.render('./temp_pages/avrms_page_temp.html')
    if dbg_mode==True:
        print("Render avrms page...")
    Page.save_resize_html('./temp_pages/avrms_page_temp.html',cfg_file='./config_json/avrms_page.json',dest='./templates/avrms_page.html')

#生成AIRMS页
def generate_airms_page(dbg_mode=False):
    if dbg_mode==True:
        print("Function:generate_airms_page")
    airms_page=Page(layout=Page.DraggablePageLayout)
    airms_page.add(
        get_airms_gauge(),
        get_airms_line()
    )
    if dbg_mode==True:
        print("Render resizable page...")
    airms_page.render('./temp_pages/airms_page_temp.html')
    if dbg_mode==True:
        print("Render airms page...")
    Page.save_resize_html('./temp_pages/airms_page_temp.html',cfg_file='./config_json/airms_page.json',dest='./templates/airms_page.html')

#生成AWATT页
def generate_awatt_page(dbg_mode=False):
    if dbg_mode==True:
        print("Function:generate_awatt_page")
    awatt_page=Page(layout=Page.DraggablePageLayout)
    awatt_page.add(
        get_awatt_gauge(),
        get_awatt_line()
    )
    if dbg_mode==True:
        print("Render resizable page...")
    awatt_page.render('./temp_pages/awatt_page_temp.html')
    if dbg_mode==True:
        print("Render awatt page...")
    Page.save_resize_html('./temp_pages/awatt_page_temp.html',cfg_file='./config_json/awatt_page.json',dest='./templates/awatt_page.html')

#生成AVAR页
def generate_avar_page(dbg_mode=False):
    if dbg_mode==True:
        print("Function:generate_avar_page")
    avar_page=Page(layout=Page.DraggablePageLayout)
    avar_page.add(
        get_avar_gauge(),
        get_avar_line()
    )
    if dbg_mode==True:
        print("Render resizable page...")
    avar_page.render('./temp_pages/avar_page_temp.html')
    if dbg_mode==True:
        print("Render avar page...")
    Page.save_resize_html('./temp_pages/avar_page_temp.html',cfg_file='./config_json/avar_page.json',dest='./templates/avar_page.html')

#生成BVRMS页
def generate_bvrms_page(dbg_mode=False):
    if dbg_mode==True:
        print("Function:generate_bvrms_page")
    bvrms_page=Page(layout=Page.DraggablePageLayout)
    bvrms_page.add(
        get_bvrms_gauge(),
        get_bvrms_line()
    )
    if dbg_mode==True:
        print("Render resizable page...")
    bvrms_page.render('./temp_pages/bvrms_page_temp.html')
    if dbg_mode==True:
        print("Render bvrms page...")
    Page.save_resize_html('./temp_pages/bvrms_page_temp.html',cfg_file='./config_json/bvrms_page.json',dest='./templates/bvrms_page.html')

#生成BIRMS页
def generate_birms_page(dbg_mode=False):
    if dbg_mode==True:
        print("Function:generate_birms_page")
    birms_page=Page(layout=Page.DraggablePageLayout)
    birms_page.add(
        get_birms_gauge(),
        get_birms_line()
    )
    if dbg_mode==True:
        print("Render resizable page...")
    birms_page.render('./temp_pages/birms_page_temp.html')
    if dbg_mode==True:
        print("Render birms page...")
    Page.save_resize_html('./temp_pages/birms_page_temp.html',cfg_file='./config_json/birms_page.json',dest='./templates/birms_page.html')

#生成BWATT页
def generate_bwatt_page(dbg_mode=False):
    if dbg_mode==True:
        print("Function:generate_bwatt_page")
    bwatt_page=Page(layout=Page.DraggablePageLayout)
    bwatt_page.add(
        get_bwatt_gauge(),
        get_bwatt_line()
    )
    if dbg_mode==True:
        print("Render resizable page...")
    bwatt_page.render('./temp_pages/bwatt_page_temp.html')
    if dbg_mode==True:
        print("Render bwatt page...")
    Page.save_resize_html('./temp_pages/bwatt_page_temp.html',cfg_file='./config_json/bwatt_page.json',dest='./templates/bwatt_page.html')

#生成BVAR页
def generate_bvar_page(dbg_mode=False):
    if dbg_mode==True:
        print("Function:generate_bvar_page")
    bvar_page=Page(layout=Page.DraggablePageLayout)
    bvar_page.add(
        get_bvar_gauge(),
        get_bvar_line()
    )
    if dbg_mode==True:
        print("Render resizable page...")
    bvar_page.render('./temp_pages/bvar_page_temp.html')
    if dbg_mode==True:
        print("Render bvar page...")
    Page.save_resize_html('./temp_pages/bvar_page_temp.html',cfg_file='./config_json/bvar_page.json',dest='./templates/bvar_page.html')

#生成CVRMS页
def generate_cvrms_page(dbg_mode=False):
    if dbg_mode==True:
        print("Function:generate_cvrms_page")
    cvrms_page=Page(layout=Page.DraggablePageLayout)
    cvrms_page.add(
        get_cvrms_gauge(),
        get_cvrms_line()
    )
    if dbg_mode==True:
        print("Render resizable page...")
    cvrms_page.render('./temp_pages/cvrms_page_temp.html')
    if dbg_mode==True:
        print("Render cvrms page...")
    Page.save_resize_html('./temp_pages/cvrms_page_temp.html',cfg_file='./config_json/cvrms_page.json',dest='./templates/cvrms_page.html')

#生成CIRMS页
def generate_cirms_page(dbg_mode=False):
    if dbg_mode==True:
        print("Function:generate_cirms_page")
    cirms_page=Page(layout=Page.DraggablePageLayout)
    cirms_page.add(
        get_cirms_gauge(),
        get_cirms_line()
    )
    if dbg_mode==True:
        print("Render resizable page...")
    cirms_page.render('./temp_pages/cirms_page_temp.html')
    if dbg_mode==True:
        print("Render cirms page...")
    Page.save_resize_html('./temp_pages/cirms_page_temp.html',cfg_file='./config_json/cirms_page.json',dest='./templates/cirms_page.html')

#生成CWATT页
def generate_cwatt_page(dbg_mode=False):
    if dbg_mode==True:
        print("Function:generate_cwatt_page")
    cwatt_page=Page(layout=Page.DraggablePageLayout)
    cwatt_page.add(
        get_cwatt_gauge(),
        get_cwatt_line()
    )
    if dbg_mode==True:
        print("Render resizable page...")
    cwatt_page.render('./temp_pages/cwatt_page_temp.html')
    if dbg_mode==True:
        print("Render cwatt page...")
    Page.save_resize_html('./temp_pages/cwatt_page_temp.html',cfg_file='./config_json/cwatt_page.json',dest='./templates/cwatt_page.html')

#生成CVAR页
def generate_cvar_page(dbg_mode=False):
    if dbg_mode==True:
        print("Function:generate_cvar_page")
    cvar_page=Page(layout=Page.DraggablePageLayout)
    cvar_page.add(
        get_cvar_gauge(),
        get_cvar_line()
    )
    if dbg_mode==True:
        print("Render resizable page...")
    cvar_page.render('./temp_pages/cvar_page_temp.html')
    if dbg_mode==True:
        print("Render cvar page...")
    Page.save_resize_html('./temp_pages/cvar_page_temp.html',cfg_file='./config_json/cvar_page.json',dest='./templates/cvar_page.html')

#生成NIRMS页
def generate_nirms_page(dbg_mode=False):
    if dbg_mode==True:
        print("Function:generate_nirms_page")
    nirms_page=Page(layout=Page.DraggablePageLayout)
    nirms_page.add(
        get_nirms_gauge(),
        get_nirms_line()
    )
    if dbg_mode==True:
        print("Render resizable page...")
    nirms_page.render('./temp_pages/nirms_page_temp.html')
    if dbg_mode==True:
        print("Render nirms page...")
    Page.save_resize_html('./temp_pages/nirms_page_temp.html',cfg_file='./config_json/nirms_page.json',dest='./templates/nirms_page.html')

#Flask框架线程函数
def flask_threads():
    if debug_mode==True:
        print("Function:flask_thread")
    app = Flask(__name__)
    app.jinja_env.auto_reload = True
    app.config['TEMPLATES_AUTO_RELOAD'] = True

    @app.route('/')
    def main_page():
        return render_template('main_page.html')

    @app.route('/main_page/')
    def main_page_back():
        return render_template('main_page.html')

    @app.route('/temp_page/')
    def temp_page():
        return render_template('temp_page.html')
    
    @app.route('/humid_page/')
    def humid_page():
        return render_template('humid_page.html')
    
    @app.route('/avrms_page/')
    def avrms_page():
        return render_template('avrms_page.html')

    @app.route('/airms_page/')
    def airms_page():
        return render_template('airms_page.html')

    @app.route('/awatt_page/')
    def awatt_page():
        return render_template('awatt_page.html')

    @app.route('/avar_page/')
    def avar_page():
        return render_template('avar_page.html')
    
    @app.route('/bvrms_page/')
    def bvrms_page():
        return render_template('bvrms_page.html')

    @app.route('/birms_page/')
    def birms_page():
        return render_template('birms_page.html')

    @app.route('/bwatt_page/')
    def bwatt_page():
        return render_template('bwatt_page.html')

    @app.route('/bvar_page/')
    def bvar_page():
        return render_template('bvar_page.html')
    
    @app.route('/cvrms_page/')
    def cvrms_page():
        return render_template('cvrms_page.html')

    @app.route('/cirms_page/')
    def cirms_page():
        return render_template('cirms_page.html')

    @app.route('/cwatt_page/')
    def cwatt_page():
        return render_template('cwatt_page.html')

    @app.route('/cvar_page/')
    def cvar_page():
        return render_template('cvar_page.html')
    
    @app.route('/nirms_page/')
    def nirms_page():
        return render_template('nirms_page.html')

    if debug_mode==True:
        print("Start Flask app...")
    app.run(host='0.0.0.0',port= 8080)#Flask在指定端口提供服务


if __name__=="__main__":
    if debug_mode==True:
        print("DEBUG MODE ON!")
    #启动UDP线程
    try:
        if debug_mode==True:
            print("Start UDP threading...")
        udp_threading=Thread(target=udp_threads,name="udp_threading")
        udp_threading.daemon = True
        udp_threading.start()
    except:
        print("Failed to start UDP threading!")
    #启动Flask线程
    try:
        if debug_mode==True:
            print("Start Flask threading...")
        udp_threading=Thread(target=flask_threads,name="flask_threading")
        udp_threading.daemon = True
        udp_threading.start()
    except:
        print("Failed to start Flask threading!")
    while True:
        generate_main_page(dbg_mode=debug_mode)
        generate_temp_page(dbg_mode=debug_mode)
        generate_humid_page(dbg_mode=debug_mode)

        generate_airms_page(dbg_mode=debug_mode)
        generate_avrms_page(dbg_mode=debug_mode)
        generate_awatt_page(dbg_mode=debug_mode)
        generate_avar_page(dbg_mode=debug_mode)

        generate_birms_page(dbg_mode=debug_mode)
        generate_bvrms_page(dbg_mode=debug_mode)
        generate_bwatt_page(dbg_mode=debug_mode)
        generate_bvar_page(dbg_mode=debug_mode)

        generate_cirms_page(dbg_mode=debug_mode)
        generate_cvrms_page(dbg_mode=debug_mode)
        generate_cwatt_page(dbg_mode=debug_mode)
        generate_cvar_page(dbg_mode=debug_mode)

        generate_nirms_page(dbg_mode=debug_mode)

        time.sleep(1)
