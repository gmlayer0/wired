#!/bin/python3
# We use a DAG to describe dataflow of controlling signals
# A -> B -> {C01->C02,C1,C2} -> D
# can be describe as follow
from functools import cmp_to_key
import os, json, sys
'''
node_relations = [
    ["C2": "D"],
    ["C1": "D"],
    ["C02": "D"],
    ["C01": "C02"],
    ["B": "C01"],
    ["B": "C1"],
    ["B": "C2"],
    ["A": "B"],
    ["Entry": "A"]
]
'''
# Each node will be generate with a structure, and each relation in json
# will generate a function for convert between structure.

class design_parser:
    def __init__(self):
        # directly read from file
        self.const_list = {}
        self.signal_list = {}    # signal_name -> {length,stage,default_value,invalid_value}
        self.node_priv_signal = {} # node_name -> list[signal_name]
        self.inst_list = {}      # inst_name   -> {opcode,signal_name: value}
        self.node_relations = [] # list[dict(node_name:node:name)]
        # generate
        self.entry_node  = '' # node_name
        self.node_dag    = {} # node_name -> list[node_name]
        self.node_signal = {} # node_name -> list[signal_name]
    
    def parse_dict(self, dict_info):
        if dict_info.get('const') is not None:
            sub_obj = dict_info.get('const')
            for pair in sub_obj:
                self.const_list[pair] = sub_obj[pair]
        if dict_info.get('signal') is not None:
            sub_obj = dict_info.get('signal')
            for signal in sub_obj:
                self.signal_list[signal] = sub_obj[signal]
                if(self.node_priv_signal.get(sub_obj[signal]['stage']) is None):
                    self.node_priv_signal[sub_obj[signal]['stage']] = []
                self.node_priv_signal[sub_obj[signal]['stage']].append(signal)
        if dict_info.get('inst') is not None:
            sub_obj = dict_info.get('inst')
            for inst in sub_obj:
                self.inst_list[inst] = sub_obj[inst]
        if dict_info.get('node_relations') is not None:
            sub_obj = dict_info.get('node_relations')
            self.node_relations = sub_obj
        return

    def gen_dag(self):
        for relation in self.node_relations:
            self.node_dag[relation[1]]    = []
        for relation in self.node_relations:
            if(relation[0] == 'Entry'):
                self.entry_node = relation[1]
                continue
            self.node_dag[relation[0]].append(relation[1])
            self.node_dag[relation[0]] += self.node_dag[relation[1]]
        for node in self.node_dag:
            node_list = self.node_dag[node]
            inst_list = []
            if(self.node_priv_signal.get(node) is not None):
                inst_list += self.node_priv_signal.get(node)
            for target in node_list:
                if(self.node_priv_signal.get(target) is not None):
                    inst_list += self.node_priv_signal.get(target)
            self.node_signal[node] = inst_list

    def gen_blank(self, times):
        return '    ' * times

    def dict_order_cmp(self,a,b):
        str_a = self.inst_list[a]['opcode']
        str_b = self.inst_list[b]['opcode']
        if len(str_a) != len(str_b):
            return (len(str_a) - len(str_b))
        str_a_std = str_a.replace('x','0')
        str_b_std = str_b.replace('x','0')
        return (int(str_a_std) - int(str_b_std))

    def gen_module(self):
        str_builder = "`include \"wired0_decoder.svh\"\n\n"
        str_builder += 'module wired_decoder(\n'
        str_builder += '    input  logic [31:0] inst_i,\n'
        str_builder += '    output logic decode_err_o,\n'
        str_builder += '    output is_t is_o\n'
        # str_builder += '    output logic[31:0][7:0] inst_string_o\n'
        str_builder += ');\n\n'
        
        # main combine logic
        depth = 1
        inst_list_dict_order = [inst_name for inst_name in self.inst_list]
        print(inst_list_dict_order)
        inst_list_dict_order.sort(key=cmp_to_key(self.dict_order_cmp))
        print(inst_list_dict_order)
        str_builder += self.gen_blank(depth) + "always_comb begin\n"
        depth += 1
        str_builder += self.gen_blank(depth) + 'decode_err_o = 1\'b1;\n'
        for signal in self.signal_list:
            signal_value = self.signal_list[signal]['default_value']
            if isinstance(signal_value,int):
                signal_value = str(self.signal_list[signal]['length']) + "\'d" + str(signal_value)
            str_builder += self.gen_blank(depth) + 'is_o.' + signal + ' = ' + signal_value + ';\n'
        # str_builder += self.gen_blank(depth) + 'inst_string_o = {' + ' ,'.join(['8\'d' + str(ord(s)) for s in 'NONEVALID']) + '}; //' + 'NONEVALID' + '\n'
        str_builder += self.gen_blank(depth) + "unique casez(inst_i)\n"
        depth += 1
        opcode_len = 0
        while len(inst_list_dict_order) != 0 and opcode_len != 32:
            if len(inst_list_dict_order) != 0 and len(self.inst_list[inst_list_dict_order[0]]['opcode']) > opcode_len:
                opcode_len += 1
                continue
            while len(inst_list_dict_order) != 0 and len(self.inst_list[inst_list_dict_order[0]]['opcode']) == opcode_len:
                inst = inst_list_dict_order[0]
                inst_list_dict_order.remove(inst)
                str_builder += self.gen_blank(depth) + "32'b" + self.inst_list[inst]['opcode'].replace('x','?') + (32 - opcode_len) * '?' + ': begin\n'
                depth += 1
                str_builder += self.gen_blank(depth) + 'decode_err_o = 1\'b0;\n'
                for signal in self.signal_list:
                    signal_value = ''
                    if self.inst_list[inst].get(signal) is not None:
                        signal_value = self.inst_list[inst].get(signal)
                    else:
                        continue
                    if isinstance(signal_value,int):
                        signal_value = str(self.signal_list[signal]['length']) + "\'d" + str(signal_value)
                    str_builder += self.gen_blank(depth) + 'is_o.' + signal + ' = ' + signal_value + ';\n'
                    # str_builder += self.gen_blank(depth) + 'is_o.' + self.signal_list[signal][0] + '.' + signal + ' = ' + signal_value + ';\n'
                # str_builder += self.gen_blank(depth) + 'inst_string_o = {' + ' ,'.join(['8\'d' + str(ord(s)) for s in inst]) + '}; //' + inst + '\n'
                # str_builder += self.gen_blank(depth) + 'inst_string_o = \'0; //' + inst + '\n' 
                depth -= 1
                str_builder += self.gen_blank(depth) + "end\n"
        # str_builder += self.gen_blank(depth) + "default: begin\n"
        # depth += 1
        # depth -= 1
        # str_builder += self.gen_blank(depth) + "end\n"

        depth -= 1
        str_builder += self.gen_blank(depth) + "endcase\n"
        depth -= 1
        str_builder += self.gen_blank(depth) + "end\n\n"
        str_builder += "endmodule\n"
        return str_builder

    def gen_header(self):
        str_builder = "`ifndef _DECODE_HEADER\n`define _DECODE_HEADER\n\n"
        # const value define
        for const_value in self.const_list:
            str_builder += '`define '
            str_builder += const_value
            str_builder += ' ('
            str_builder += self.const_list[const_value]
            str_builder += ')\n'
        str_builder += '\n'

        # signal struct define
        for leaf_struct in self.signal_list:
            str_builder += 'typedef logic ['
            str_builder += str(self.signal_list[leaf_struct]['length'] - 1)
            str_builder += ' : 0] '
            str_builder += leaf_struct
            str_builder += '_t;\n'
        str_builder += '\n'

        # node defination
        for node in self.node_signal:
            str_builder += 'typedef struct packed {\n'
            for signal_name in self.node_signal[node]:
                str_builder += '    ' + signal_name + '_t ' + signal_name + ';\n'
            str_builder += '} decode_info_' + node + '_t;\n\n'

        # convert function defination
        for relation in self.node_relations:
            from_stage = relation[0]
            to_stage   = relation[1]
            if from_stage == 'Entry':
                continue
            str_builder += 'function automatic decode_info_' + to_stage + '_t '
            # [name]([type] [value])
            str_builder += 'get_' + to_stage + '_from_' + from_stage + '(input decode_info_' + from_stage + '_t ' + from_stage + ');\n' 
            # <statement>
            str_builder += '    decode_info_' + to_stage + '_t ' + 'ret;\n'
            for signal_name in self.node_signal[to_stage]:
                str_builder += '    ret.' + signal_name + ' = ' + from_stage + '.' + signal_name + ';\n'
            str_builder += '    return ret;\n'
            # endfunction
            str_builder += 'endfunction\n\n'
        str_builder += "`endif\n"
        return str_builder

    def parse_all_file(self):
        json_path_list = []
        for root, dirs, files in os.walk('.'):
            for file in files:
                path = os.path.join(root, file)
                if os.path.splitext(path)[1] == '.json':
                    print(path)
                    json_path_list.append(path)
        for json_file_path in json_path_list:
            f = open(json_file_path) # -> with open
            file_info = json.load(f)
            self.parse_dict(file_info)
            f.close()
        self.gen_dag()


if __name__ == '__main__':
    parser = design_parser()
    parser.parse_all_file()
    # f = open('decoder.sv','w')
    # f.write(parser.gen_sv_module())
    # f.close()
    f = open('wired0_decoder.svh','w')
    f.write(parser.gen_header())
    f.close()
    f = open('wired2_decoder.sv','w')
    f.write(parser.gen_module())
    f.close()
    # -m : move decoder directly to rtl/decoder/
    if len(sys.argv) > 1:
        op = sys.argv[1]
        if op == '-m' or op == '--move':
            cur_file_path = os.path.split(os.path.realpath(__file__))[0]
            tar_path = os.path.join(cur_file_path, '../../rtl/decoder/')
            os.system("cp decoder.sv " + tar_path)
            print("cp decoder.sv " + tar_path)
            os.system("cp decoder.svh "+ tar_path)
            print("cp decoder.svh "+ tar_path)
