import pyecharts.options as opts
from pyecharts.charts import Line

temperature = [11, 11, 15, 13, 12, 13, 10,11,12,6,5,4,3,2,1,2,3,4,5,6,7,7,8,8,8]

temp_line=(
    Line()
    .add_xaxis(xaxis_data=[str(m) for m in range(1,len(temperature)+1)])
    .add_yaxis(
        series_name="温度(℃)",
        y_axis=temperature,
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
        title_opts=opts.TitleOpts(title="温度检测", subtitle="历史数据"),
        tooltip_opts=opts.TooltipOpts(trigger="axis"),
        toolbox_opts=opts.ToolboxOpts(is_show=True),
        xaxis_opts=opts.AxisOpts(name="记录",type_="category", boundary_gap=False),
        yaxis_opts=opts.AxisOpts(name="温度(℃)"),
    )   
)
temp_line.render("temperature_change_line_chart.html")
