def configure_styles(style_obj, theme="default"):
    """Cấu hình kiểu giao diện cho GUI."""
    style_obj.configure('TLabel', font=('Helvetica', 12), foreground='#4682B4', background='#E6F3FA')
    style_obj.configure('TEntry', font=('Helvetica', 10))
    style_obj.configure('TButton', font=('Helvetica', 11, 'bold'), foreground='white', background='#1E90FF')
    style_obj.map('TButton', background=[('active', '#1C86EE')])
    style_obj.configure('Stop.TButton', font=('Helvetica', 12, 'bold'), foreground='white', background='#FF4500')
    style_obj.map('Stop.TButton', background=[('active', '#EE4000')])
    style_obj.configure('TLabelFrame', font=('Helvetica', 12, 'bold'), foreground='#4682B4', background='#E6F3FA')
    style_obj.configure('TLabelFrame.Label', font=('Helvetica', 12, 'bold'), foreground='#4682B4', background='#E6F3FA')